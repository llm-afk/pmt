"""Local adapter diagnostics and production MES reachability."""
import json
import os
from pathlib import Path
import subprocess
import threading
import time


class NetworkLost(RuntimeError):
    diagnostic_code = 'PRODUCTION_NETWORK_LOST'
    diagnostic_hint = '本件尚未通过测试。恢复网络后重新测试。'


def mes_reachable(url):
    from http.client import HTTPException
    from urllib.error import HTTPError
    from urllib.parse import urlsplit
    from urllib.request import Request, ProxyHandler, build_opener
    from .integrations import _NoRedirect
    # HEAD on the configured base uses the same system proxy/TLS path as MES.
    # Even 401/404/405 proves a reachable server; do not call a business endpoint.
    try:
        if urlsplit(url).scheme not in ('http','https'):
            return False
        handlers = [ProxyHandler({})] if urlsplit(url).hostname in ('127.0.0.1','localhost','::1') else []
        with build_opener(*handlers, _NoRedirect()).open(Request(url, method='HEAD'), timeout=2):
            return True
    except HTTPError as exc:
        exc.close()
        return exc.code not in (407, 502, 503, 504)
    except (OSError, ValueError, HTTPException):
        return False


MES_POLL_S = 2
MES_LOSS_GRACE_S = 12
MES_MIN_FAILURES = 3


class ProductionNetwork:
    def __init__(self, endpoint, shutdown, notify, probe=mes_reachable, *, clock=time.monotonic):
        self.endpoint, self.shutdown, self.notify, self.probe = endpoint, shutdown, notify, probe
        self.clock = clock
        self.lock = threading.RLock()
        self.poll_lock = threading.Lock()
        self.url, self.online, self.checked, self.epoch = '', None, None, 0
        self.failed_since, self.failures = None, 0
        self.sample_count, self.last_probe_ok = 0, False
        self.thread = None

    def refresh(self):
        # A start click and the watcher can share a sample taken while both were waiting.
        with self.lock:
            previous_count = self.sample_count
        with self.poll_lock:
            url = self.endpoint()
            with self.lock:
                if previous_count != self.sample_count and self.url == url:
                    return self.last_probe_ok
            ok = bool(url and self.probe(url))
            with self.lock:
                if self.endpoint() != url:
                    return False
                if self.url != url:
                    self.url, self.online = url, None
                    self.failed_since, self.failures = None, 0
                    self.epoch += 1
                previous = self.online
                now = self.clock()
                self.checked, self.last_probe_ok = now, ok
                self.sample_count += 1
                if ok:
                    self.failed_since, self.failures = None, 0
                    self.online = True
                else:
                    if self.failed_since is None:
                        self.failed_since = now
                    self.failures += 1
                    if self.failures >= MES_MIN_FAILURES and now-self.failed_since >= MES_LOSS_GRACE_S:
                        self.online = False
                changed = self.online != previous
                if changed and self.online is False:
                    self.epoch += 1
                online = self.online
            if changed:
                self.notify(online)
            return ok

    def state(self):
        with self.lock:
            if self.url != self.endpoint() or self.online is None or self.clock()-self.checked > 10:
                return 'CHECKING'
            return 'READY' if self.online else 'OFFLINE'

    def ready(self):
        return self.state() == 'READY'

    def verify(self):
        # Entry and PASS need an actual successful sample. Wait out an unsettled
        # failure on the station worker, never on the UI or acquisition thread.
        while not self.shutdown.is_set():
            if self.refresh():
                return True
            if self.state() == 'OFFLINE':
                return False
            self.shutdown.wait(MES_POLL_S)
        raise InterruptedError('工位已关闭，停止核对 MES 网络。')

    def start(self):
        def watch():
            while not self.shutdown.is_set():
                self.refresh()
                self.shutdown.wait(MES_POLL_S)
        self.thread = threading.Thread(target=watch, name='production-network', daemon=True)
        self.thread.start()

    def begin(self, *, resume=False):
        if not resume and not self.verify():
            raise NetworkLost('MES 网络持续不可达，未开始测试。')
        if resume:
            self.refresh()
        with self.lock:
            if not resume and not self.ready():
                raise NetworkLost('MES 网络状态已变化，未开始测试。')
            return OnlineWorkpiece(self, self.epoch)


class OnlineWorkpiece:
    def __init__(self, network, epoch):
        self.network, self.epoch = network, epoch
        self.committed = self.lost = False

    def check(self):
        with self.network.lock:
            if self.committed:
                return
            if (self.lost or self.network.epoch != self.epoch or self.network.online is False
                    or self.network.url != self.network.endpoint()):
                self.lost = True
                raise NetworkLost('测试通过前 MES 网络中断，本次测试数据报废。')

    def confirm_pass(self, persist):
        if not self.network.verify():
            self.lost = True
        with self.network.lock:
            self.check()
            persist()
            self.committed = True


NETWORK_INTERVAL_S = 30
NETWORK_TIMEOUT_S = 5
# .NET reads adapter properties locally. No ping, DNS lookup, HTTP request, SSID or key collection.
NETWORK_COMMAND = r"""
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = New-Object System.Text.UTF8Encoding($false)
$cards = @([System.Net.NetworkInformation.NetworkInterface]::GetAllNetworkInterfaces() | ForEach-Object {
    $card = $_
    $kind = $card.NetworkInterfaceType.ToString()
    if ($kind -ne 'Loopback' -and $kind -ne 'Tunnel') {
        $properties = $null
        $interfaceError = ''
        try { $properties = $card.GetIPProperties() } catch { $interfaceError = $_.Exception.Message }
        [pscustomobject]@{
            name = $card.Name
            description = $card.Description
            kind = $kind
            state = $card.OperationalStatus.ToString()
            addresses = @($properties.UnicastAddresses | ForEach-Object { if ($_ -ne $null) { $_.Address.ToString() } })
            gateways = @($properties.GatewayAddresses | ForEach-Object { if ($_ -ne $null) { $_.Address.ToString() } })
            dns = @($properties.DnsAddresses | ForEach-Object { if ($_ -ne $null) { $_.ToString() } })
            error = $interfaceError
        }
    }
})
[pscustomobject]@{
    available = [System.Net.NetworkInformation.NetworkInterface]::GetIsNetworkAvailable()
    interfaces = $cards
} | ConvertTo-Json -Depth 6 -Compress
"""


def spawn_snapshot():
    windows = Path(os.environ.get('SystemRoot', r'C:\Windows'))
    return subprocess.run([str(windows/'System32/WindowsPowerShell/v1.0/powershell.exe'),
        '-NoLogo','-NoProfile','-NonInteractive','-Command',NETWORK_COMMAND],
        stdin=subprocess.DEVNULL,capture_output=True,text=True,encoding='utf-8',errors='replace',
        creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0),timeout=NETWORK_TIMEOUT_S,check=True)


def read_network_snapshot():
    if os.name!='nt':
        raise OSError('本机网络配置检查目前仅支持 Windows')
    output = spawn_snapshot().stdout.lstrip('\ufeff').strip()
    snapshot = json.loads(output)
    if (not isinstance(snapshot,dict) or type(snapshot.get('available')) is not bool
            or not isinstance(snapshot.get('interfaces'),list)):
        raise ValueError('系统网络配置未返回预期格式')
    for card in snapshot['interfaces']:
        if (not isinstance(card,dict) or not all(isinstance(card.get(key),str) for key in ('name','kind','state'))
                or not all(isinstance(card.get(key),list) and all(isinstance(item,str) for item in card[key])
                           for key in ('addresses','gateways','dns'))):
            raise ValueError('系统网络接口信息不完整')
        for key in ('addresses','gateways','dns'):
            card[key] = sorted(set(card[key]))
    snapshot['interfaces'].sort(key=lambda item:(item['kind'],item['name']))
    return snapshot


def network_messages(snapshot):
    if snapshot.get('error'):
        return [('WARNING','网络状态检查失败：'+snapshot['error']+'；详情已记入后台日志。')]
    cards = snapshot['interfaces']
    messages = []
    for card in cards[:8]:
        if card.get('error'):
            messages.append(('WARNING',card['name']+'：读取接口配置失败；'+card['error']))
            continue
        kind = ('Wi-Fi' if card['kind']=='Wireless80211' else
                '以太网' if 'Ethernet' in card['kind'] else '网络接口')
        title = kind+' · '+card['name']
        if card['state']=='Up':
            ip = '、'.join(card['addresses'][:3]) or '尚未获得 IP'
            gateway = '、'.join(card['gateways'][:2]) or '未检测到默认网关'
            dns = '、'.join(card['dns'][:2]) or '未配置'
            messages.append(('INFO',f'{title}：链路已连接；IP {ip}；网关 {gateway}；DNS {dns}'))
        else:
            messages.append(('INFO',f'{title}：未连接（{card["state"]}）'))
    if len(cards)>8:
        messages.append(('INFO',f'另有 {len(cards)-8} 个接口，完整地址与 DNS 已记录到后台日志。'))
    if not any(card['state']=='Up' and card['addresses'] for card in cards):
        messages.append(('WARNING','未发现已连接且有 IP 的网络接口，请检查 Wi-Fi 或网线。'))
    messages.append(('INFO','本机网络检查完成；MES / OSS 可用性及鉴权以实际请求回执为准。'))
    return messages


class NetworkMonitor:
    def __init__(self, emit, stop, diagnostics, *, probe=read_network_snapshot, interval=NETWORK_INTERVAL_S):
        self.emit, self.stop, self.diagnostics = emit, stop, diagnostics
        self.probe, self.interval = probe, interval
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self.run,name='network-status',daemon=True)
        self.thread.start()

    def run(self):
        previous = None
        while not self.stop.is_set():
            started = time.monotonic()
            try:
                snapshot = self.probe()
                error = None
            except Exception as exc:
                if isinstance(exc,subprocess.TimeoutExpired):
                    message=f'读取系统网络配置超时（{NETWORK_TIMEOUT_S} 秒）'
                elif isinstance(exc,subprocess.CalledProcessError):
                    message=f'系统网络查询失败（退出码 {exc.returncode}）：'+str(exc.stderr or '').strip()[:240]
                else:
                    message=str(exc) or type(exc).__name__
                snapshot, error = dict(error=message), exc
            if self.stop.is_set():
                return
            signature = json.dumps(snapshot,sort_keys=True,ensure_ascii=False)
            if signature != previous:
                self.diagnostics.station.event('environment.network.changed','本机网络状态',
                    level='WARNING' if error else 'INFO', error=error,snapshot=snapshot,
                    local_only=True,cloud_verified=False,stderr=str(getattr(error,'stderr','') or '')[:4096],
                    elapsed_s=round(time.monotonic()-started,3))
                self.emit(snapshot)
                previous = signature
            else:
                self.diagnostics.station.sample('environment.network.heartbeat',interval=300,
                    snapshot=snapshot,local_only=True,cloud_verified=False)
            self.stop.wait(self.interval)

"""Install a verified PMT main release alongside field-owned station data and firmware."""
from contextlib import closing
import hashlib
import json
import os
from pathlib import Path, PureWindowsPath
import re
import shutil
import stat
import subprocess
import sys
import time
from urllib.parse import quote
from urllib.request import Request, urlopen

from .lifecycle import AlreadyRunning, StationInstance


EXE = '电机产测软件.exe'
MANIFEST = 'release-manifest.json'
WORK = '.pmt-update'
OWNER = 'PMT software updater 1\n'
REF = 'refs/heads/main'
TRACKING = 'refs/remotes/origin/main'
API = 'https://api.github.com/repos/llm-afk/pmt/commits/main'
RAW = 'https://raw.githubusercontent.com/llm-afk/pmt/'
ROOT_FILES = {EXE, '.gitattributes', '.gitignore', 'README.md', 'install_drivers.cmd', 'pyproject.toml'}
PACKAGE_DIRS = {'docs', 'drivers', 'motor_platform', 'scripts', 'services', 'tests'}
BLOCK = 1024 * 1024


class UpdateError(RuntimeError):
    pass


def version_number(value):
    if not isinstance(value, str) or not re.fullmatch(r'\d{1,8}\.\d{1,8}\.\d{1,8}', value, re.ASCII):
        raise UpdateError('发布清单中的软件版本无效。')
    return tuple(int(part) for part in value.split('.'))


def sha(path):
    if not path.exists():
        return None
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def package_path(root, name):
    parts = name.split('/')
    if (not (name in ROOT_FILES or name == MANIFEST or len(parts) > 1 and parts[0] in PACKAGE_DIRS)
            or any(not p or p.startswith('.') and p not in {'.gitattributes','.gitignore','.bin','.package-lock.json','.releaserc'}
                   or p.endswith(('.', ' ')) or PureWindowsPath(p).is_reserved()
                   or re.search(r'[\\:<>"|?*\x00-\x1f]', p) for p in parts)):
        raise UpdateError('软件包包含无效路径：'+name)
    target = root
    for part in parts:
        target /= part
        reject_link(target)
    return target


def reject_link(path):
    if path.is_symlink() or (path.exists() and
            getattr(path.lstat(), 'st_file_attributes', 0) & stat.FILE_ATTRIBUTE_REPARSE_POINT):
        raise UpdateError('更新路径含有文件链接：'+str(path))


def read_release(data, root):
    m = json.loads(data)
    if (m.get('project'), m.get('channel'), m.get('executable')) != ('pmt', 'production', EXE):
        raise UpdateError('所选目录或远端清单不是 PMT 正式发布包。')
    version_number(m.get('version'))
    files = m.get('files')
    if not isinstance(files, dict) or files.get(EXE) != m.get('sha256') or EXE not in files:
        raise UpdateError('正式发布包缺少主程序指纹。')
    keys = set()
    for name, digest in files.items():
        package_path(root, name)
        if name == MANIFEST or name.casefold() in keys or not re.fullmatch('[0-9a-f]{64}', str(digest)):
            raise UpdateError('发布文件清单的路径或指纹无效。')
        keys.add(name.casefold())
    return m


def git(root, *args, out=subprocess.PIPE):
    env = dict(os.environ, GIT_TERMINAL_PROMPT='0', GCM_INTERACTIVE='Never')
    try:
        proc = subprocess.run(['git','-C',str(root),*args], stdin=subprocess.DEVNULL,
            stdout=out, stderr=subprocess.PIPE, timeout=180, env=env,
            creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    except FileNotFoundError as exc:
        raise UpdateError('此目录由 Git 克隆，请先安装 Git for Windows 后再点更新。') from exc
    if proc.returncode:
        detail = proc.stderr.decode('utf-8', 'replace').strip()[-500:]
        raise UpdateError('Git 操作未完成。'+detail)
    return proc.stdout


def repository_state(root):
    if git(root,'symbolic-ref','-q','HEAD').strip() != REF.encode():
        raise UpdateError('此目录未检出 main，请由工程人员核对分支。')
    head = git(root,'rev-parse','HEAD').decode().strip()
    return head, git(root,'write-tree').strip()


def tree(root, rev):
    return git(root,'rev-parse',rev+'^{tree}').strip()


def http(url):
    return urlopen(Request(url, headers={'User-Agent':'PMT-station-updater',
                                         'Accept':'application/octet-stream' if url.startswith(RAW) else 'application/json'}), timeout=25)


class ReleaseSource:
    def __init__(self, root):
        self.root = root
        self.base = ''
        if (root/'.git').exists():
            origin = git(root,'remote','get-url','origin').decode().strip().lower().removesuffix('/').removesuffix('.git')
            if origin not in {'https://github.com/llm-afk/pmt','git@github.com:llm-afk/pmt',
                              'ssh://git@github.com/llm-afk/pmt'}:
                raise UpdateError('当前 origin 与 PMT 正式仓库不一致，请工程人员核对。')
            self.base, index = repository_state(root)
            if index != tree(root,self.base):
                raise UpdateError('此目录有尚未提交的暂存改动，请工程人员处理后再更新。')
            git(root, 'fetch', '--no-tags', 'origin', REF+':'+TRACKING)
            self.commit = git(root,'rev-parse',TRACKING).decode().strip()
            git(root,'merge-base','--is-ancestor',self.base,self.commit)
        else:
            with closing(http(API)) as response:
                self.commit = json.loads(response.read(BLOCK))['sha']
        if not re.fullmatch('[0-9a-f]{40}',self.commit):
            raise UpdateError('未能确认 main 的发布提交。')

    def download(self, name, dest):
        if self.base:
            blob = self.commit+':'+name
            if name == MANIFEST:
                dest.write(git(self.root,'cat-file','blob',blob))
            else:
                git(self.root,'cat-file','blob',blob,out=dest)
        else:
            with closing(http(RAW+self.commit+'/'+quote(name))) as response:
                shutil.copyfileobj(response,dest,BLOCK)


def workspace(root, *, create=False):
    work = root/WORK
    reject_link(work)
    marker = work/'owner'
    if not work.exists() and create:
        work.mkdir()
        marker.write_text(OWNER, encoding='ascii')
    reject_link(marker)
    if not marker.is_file() or marker.read_text(encoding='ascii') != OWNER:
        raise UpdateError('更新缓存目录异常，请工程人员检查 '+str(work))
    return work


def save_plan(work, plan):
    from .station_config import atomic_json
    atomic_json(work/'plan.json',plan)


def load_plan(work):
    return json.loads((work/'plan.json').read_text(encoding='utf-8'))


def prepare(root, current, progress, cancelled, *, source_factory=ReleaseSource):
    from io import BytesIO
    old_data = (root/MANIFEST).read_bytes()
    old = read_release(old_data,root)
    if old['version'] != current:
        raise UpdateError('运行中的版本与根目录发布清单不一致，请重新打开完整发布包。')
    progress('正在连接 PMT 正式仓库…')
    source = source_factory(root)
    data = BytesIO()
    source.download(MANIFEST,data)
    new_data = data.getvalue()
    new = read_release(new_data,root)
    if version_number(new['version']) <= version_number(current):
        return None
    if source.base:
        if git(root,'cat-file','blob',source.base+':'+MANIFEST) != old_data:
            raise UpdateError('本地发布清单有修改，请工程人员核对。')
        changed = git(root,'diff','--name-only','--no-renames','-z',source.base,source.commit).decode().split('\0')
        owned = old['files'].keys() | new['files'].keys() | {MANIFEST}
        if any(n and n not in owned and not n.startswith('firmware/') for n in changed):
            raise UpdateError('新版本改变了发布包结构，请使用完整发布包更新。')
    work = workspace(root,create=True)
    with StationInstance(work,lock_name='update.lock'):
        if (work/'plan.json').is_file() and load_plan(work)['state'] in ('installing','rollback'):
            raise UpdateError('上次安装尚待恢复，请关闭并重新打开上位机。')
        payload = work/'files'
        reject_link(payload)
        if payload.exists():
            shutil.rmtree(payload)
        payload.mkdir()
        changes = []
        names = old['files'].keys() | new['files'].keys()
        for name in sorted(names, key=lambda n:(n == EXE,n)):
            before, after = old['files'].get(name), new['files'].get(name)
            if before != after:
                changes.append([name,before,after])
        changes.append([MANIFEST,hashlib.sha256(old_data).hexdigest(),hashlib.sha256(new_data).hexdigest()])
        for i,(name,before,after) in enumerate(changes):
            if cancelled():
                raise UpdateError('更新已取消。')
            path = package_path(root,name)
            actual = sha(path)
            if actual != before:
                raise UpdateError('本地文件与已安装版本不同，请工程人员核对后再更新：'+name)
            progress(f'正在准备 v{new["version"]} · {i+1}/{len(changes)} · {name}')
            if before:
                shutil.copyfile(path,payload/f'{i}.old')
                if sha(payload/f'{i}.old') != before:
                    raise UpdateError('备份校验失败：'+name)
            if after:
                downloaded = payload/f'{i}.new'
                with downloaded.open('wb') as dest:
                    if name == MANIFEST:
                        dest.write(new_data)
                    else:
                        source.download(name,dest)
                if sha(downloaded) != after:
                    raise UpdateError('下载文件 SHA-256 校验失败：'+name)
        helper = work/'installer.exe'
        reject_link(helper)
        shutil.copyfile(root/EXE,helper)
        if sha(helper) != old['files'][EXE]:
            raise UpdateError('当前主程序校验失败，请重新获取正式发布包。')
        plan = dict(state='prepared',version=new['version'],base=source.base,commit=source.commit,changes=changes)
        save_plan(work,plan)
        return new['version']


def sync_index(root, source, dest):
    head,index = repository_state(root)
    if head not in (source,dest) or index not in (tree(root,source),tree(root,dest)):
        raise UpdateError('安装期间 Git 分支或暂存区已被修改，请工程人员核对。')
    if index != tree(root,dest):
        # Two-tree merge updates the clean index only. Files were installed and checked separately.
        git(root,'read-tree','-i','-m',source,dest)
    if head != dest:
        git(root,'update-ref','-m','PMT software update',REF,dest,source)


def replace_file(path, content):
    import tempfile
    path.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=path.parent,prefix='.pmt-',suffix='.tmp',delete=False) as tmp:
        temp = Path(tmp.name)
    try:
        if content:
            shutil.copyfile(content,temp)
        deadline = time.monotonic()+12
        while True:
            try:
                if content:
                    os.replace(temp,path)
                else:
                    path.unlink()
                return
            except PermissionError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.2)
    finally:
        temp.unlink(missing_ok=True)


def install_files(root, work, plan, *, backwards):
    entries = list(enumerate(plan['changes']))
    for i,(name,before,after) in reversed(entries) if backwards else entries:
        expected, target = (after,before) if backwards else (before,after)
        path = package_path(root,name)
        actual = sha(path)
        if actual == target:
            continue
        if actual != expected:
            raise UpdateError('安装期间文件被修改：'+name)
        content = work/'files'/f'{i}.{"old" if backwards else "new"}' if target else None
        if content and sha(content) != target:
            raise UpdateError('暂存文件校验失败：'+name)
        replace_file(path,content)
        if sha(path) != target:
            raise UpdateError('文件写入后校验失败：'+name)
    if plan['base']:
        old,new = plan['base'],plan['commit']
        sync_index(root,new,old) if backwards else sync_index(root,old,new)


def install(root):
    work = workspace(root)
    with StationInstance(work,lock_name='update.lock'):
        plan = load_plan(work)
        state = plan['state']
        if state not in ('prepared','installing','rollback'):
            raise UpdateError('没有可安装的更新。')
        if state == 'prepared':
            if plan['base']:
                head,index = repository_state(root)
                if head != plan['base'] or index != tree(root,head):
                    raise UpdateError('准备安装期间 Git 已变化，请重新检查更新。')
            for name,before,_ in plan['changes']:
                if sha(package_path(root,name)) != before:
                    raise UpdateError('准备安装期间本地文件有修改：'+name)
        if state != 'rollback':
            plan['state'] = 'installing'
            save_plan(work,plan)
            try:
                install_files(root,work,plan,backwards=False)
            except Exception as exc:
                plan.update(state='rollback',error=str(exc))
                save_plan(work,plan)
            else:
                plan['state'] = 'done'
                save_plan(work,plan)
                return plan['version']
        install_files(root,work,plan,backwards=True)
        plan['state'] = 'restored'
        save_plan(work,plan)
        raise UpdateError('已恢复之前的软件版本。'+plan.get('error',''))


def spawn(executable, *args):
    return subprocess.Popen([str(executable),*args],cwd=executable.parent,
        env=dict(os.environ,PYINSTALLER_RESET_ENVIRONMENT='1'),close_fds=True,
        creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))


def resume_if_needed(root):
    if not (root/WORK/'plan.json').is_file():
        return False
    work = workspace(root)
    if load_plan(work)['state'] not in ('installing','rollback'):
        return False
    spawn(work/'installer.exe','--apply-update')
    return True


def consume_result(root):
    """Discard verified backups after restarting, retaining recovery data for interrupted installs."""
    if not (root/WORK/'plan.json').is_file():
        return ''
    work = workspace(root)
    with StationInstance(work,lock_name='update.lock'):
        plan = load_plan(work)
        if plan['state'] not in ('done','restored'):
            return ''
        message = ('已更新到 v'+plan['version'] if plan['state']=='done'
                   else '已恢复旧版：'+plan.get('error',''))
        payload = work/'files'
        reject_link(payload)
        if payload.exists():
            shutil.rmtree(payload)
        (work/'installer.exe').unlink(missing_ok=True)
        (work/'plan.json').unlink()
        return message


def apply_from_helper(executable, station_root, notice):
    work = executable.parent
    root = work.parent
    if executable.name != 'installer.exe' or work.name != WORK or workspace(root) != work:
        raise UpdateError('更新程序位置不正确。')
    deadline = time.monotonic()+45
    while True:
        try:
            with StationInstance(station_root):
                try:
                    install(root)
                except Exception as exc:
                    notice('更新未完成。\n'+str(exc))
                    if load_plan(work)['state'] in ('installing','rollback'):
                        return False
                break
        except AlreadyRunning:
            if time.monotonic() >= deadline:
                notice('原上位机仍在运行。请正常关闭后再点更新。')
                return False
            time.sleep(.25)
    spawn(root/EXE)
    return True

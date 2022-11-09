Import("env")
import os, re
import hashlib
try:
    import git
except ImportError:
    env.Execute('"$PYTHONEXE" -m pip install GitPython')
    try:
        import git
    except ImportError:
        git = None


#print(env.Dump())


def find_build_flag(search):
    if not search:
        return
    for flag in env['BUILD_FLAGS']:
        if search in flag:
            return flag
    return ""


def encode_c_string(s):
    # print(">>>>: %s" % (s))
    try:
        s = eval(s)
        if not isinstance(s, str):
            return str(s)
    except SyntaxError:
        pass
    # first remove quotes
    if s.startswith('"') and s.endswith('"'):
        s = s[1:-1]
    if s.startswith('\\"') and s.endswith('\\"'):
        s = s[1:-1]
    # encode characters
    def process_chars(val):
        convert = ['\\', '"', '!', '{', '}', '(', ')', '%', '&']
        result = ''
        for c in val:
            if not (32 <= ord(c) < 127) or c in convert:
                result += '\\\\\\\\%03o' % ord(c)
            else:
                result += c
        return result
    s = process_chars(s)
    # Quote encoded string
    output = '\\"' + s + '\\"'
    # print("<<<<: %s" % output)
    return output


def parse_flags(path):
    build_flags = env['BUILD_FLAGS']
    try:
        with open(path, "r") as _f:
            for line in _f:
                define = line.strip()
                if define and not define.startswith("#"):
                    # remove trailing comments
                    comment = define.find("#")
                    if 0 <= comment:
                        define = define[:comment].strip()
                    assign = define.find("=")
                    value = ""
                    if 0 <= assign:
                        value = define[assign+1:]
                        define = define[:assign]
                        value = "=\"%s\"" % encode_c_string(value)
                    build_flags.append(f"-D{define}{value}")
    except IOError:
        pass


def parse_env_defines():
    build_flags = env['BUILD_FLAGS']
    env_flags = env.get('ENV', {}).get('MQTT_CLIENT_FIRMWARE_FLAGS', "")
    if env_flags:
        env_flags = env_flags.split()
        for flag in env_flags:
            build_flags.append(flag)


# try to parse user private params
parse_flags("private_flags.txt")
parse_env_defines()

# Parse version information
sha_string = "development"
sha = None
if git:
    try:
        git_repo = git.Repo(os.path.abspath(os.getcwd()),
                            search_parent_directories=False)
        git_root = git_repo.git.rev_parse("--show-toplevel")
        # git describe --match=NeVeRmAtCh --always --abbrev=6 --dirty
        sha_string = git.Repo(git_root).git.describe(
            '--dirty', '--abbrev=6', '--always', '--match=NeVeRmAtCh')
        if 'dirty' in sha_string:
            env['BUILD_FLAGS'].append("-DLATEST_COMMIT_DIRTY=1")
        sha = ",".join(["0x%s" % x for x in sha_string[:6]])
    except (git.GitCommandError, git.InvalidGitRepositoryError) as err:
        # print(f"Git parsing error: '{err}'")
        pass
if sha is None:
    if os.path.exists("VERSION"):
        with open("VERSION") as _f:
            data = _f.readline()
            _f.close()
        if "$Format:%h$" not in data:
            sha_string = data.split()[1].strip()
            sha = ",".join(["0x%s" % x for x in sha_string[:6]])
if sha is None:
    sha = "0,0,0,0,0,0"

print("sha_string: '%s'" % sha_string)
print("Current SHA: %s" % sha)
env['BUILD_FLAGS'].append(f"-DLATEST_COMMIT={sha}")
env['BUILD_FLAGS'].append(f'-DLATEST_COMMIT_STR="\\"{sha_string}\\""')

print("\n[INFO] build flags: %s\n" % env['BUILD_FLAGS'])

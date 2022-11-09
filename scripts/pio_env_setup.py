Import("env", "projenv")
import esp_compress
import subprocess, os


myDNS = "my_changer"


def on_upload_ota(source, target, env):
    source_file = str(source[0])
    # Default addresses
    upload_addr = [myDNS+'.local', myDNS]
    # Upload address can be given as a --upload-port
    upload_port = env.get('UPLOAD_PORT', None)
    if upload_port is not None:
        upload_addr = [upload_port]

    # Check if the binary exits
    if not os.path.exists(source_file):
        raise SystemExit("No valid binary found!")

    cmd = ["curl", "--max-time", "60",
           "--retry", "2", "--retry-delay", "1",
           "-F", "data=@%s" % (source_file,)]

    for addr in upload_addr:
        addr = "http://%s/update" % (addr,)
        print(" ** UPLOADING TO: %s" % addr)
        try:
            subprocess.check_call(cmd + [addr])
            print()
            print("** UPLOAD SUCCESS. Flashing in progress.")
            return
        except subprocess.CalledProcessError:
            print("FAILED!")
    raise SystemExit("WIFI upload FAILED!")


env.AddCustomTarget("wifi",
    ["$BUILD_DIR/${PROGNAME}.bin"],
    [esp_compress.compressFirmware, on_upload_ota],
    title="Upload via WiFi", description="")

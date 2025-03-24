# Fix Datetime Resolve

1. Edit `sudo nvim /etc/resolv.conf` and change the `127.0.0.1` to `1.1.1.1`.
2. Sync time server with `sudo ntpdate -u time.google.com`

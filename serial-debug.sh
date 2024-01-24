alias ddcmd='echo $* > /proc/dynamic_debug/control'
alias serial-nodebug='ddcmd module serial -p'
alias serial-debug='ddcmd module serial +p'
alias mdbg="mount -t debugfs none /sys/kernel/debug"



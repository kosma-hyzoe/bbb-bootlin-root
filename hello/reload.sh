rmmod hello_version &>/dev/null
[ $# -gt 0 ] && insmod hello_version.ko whom=$1 || insmod hello_version.ko 

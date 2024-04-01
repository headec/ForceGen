opt=pre_aging
target=aging_test
if [ "$1" != "" ]; then
	opt=$1
fi
if [ "$2" != "" ]; then
	target=$2
fi
	q++ ../src/$target.cpp -L /Users/yong/force_dimension/sdk/qnx710/sdk-3.16.0/lib/release/qnx-x86_64-qcc-8.3.0 -I /Users/yong/force_dimension/sdk/qnx710/sdk-3.16.0/externals/Eigen -I /Users/yong/force_dimension/sdk/qnx710/sdk-3.16.0/include -ldrd -ldhd -lsocket -lusbdi -o $opt # local path; fix to use
mv $opt bin/
if [ "$3" != "-i" ]; then
	scp ./bin/$opt root@192.168.0.10:/root
fi

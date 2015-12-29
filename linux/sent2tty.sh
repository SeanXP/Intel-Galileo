#################################################################
#          Copyright (C) 2014 All rights reserved.
#														  
#	> File Name:        < sent2ttyS0.sh >
#	> Author:           < Shawn Guo >		
#	> Mail:             < iseanxp@gmail.com >		
#	> Created Time:     < 2014/04/12 >
#	> Last Changed: 
#	> Description:      定时向串口发送数据, 串口调试用.
#################################################################

#!/bin/bash

if [ $# == 1 ];then
    echo "send message to $1"
    dev_tty=$1
else
    echo "Usage: sh $0 /dev/tty_name"
    exit 1
fi


for i in $( seq 1 100 )
do
    echo $i > $dev_tty
    sleep 1
done

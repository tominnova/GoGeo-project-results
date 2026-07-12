#if ! test -f pipe.8b; then
#    mkfifo pipe.8b
#fi

rm -f pipe.csv

fsample=4400000
echo F_SAMPLE = ${fsample}

while true
do
    rm -f pipe.8b
    mkfifo pipe.8b
    #~/GNSS/gps-sdr-sim-master/gps-sdr-sim -b 8 -e brdc3540.14n -c 3655463.659,1404112.314,5017924.853 -d 86400 -o pipe.8b -s 4092000
    #~/GNSS/gps-sdr-sim-master/gps-sdr-sim -b 8 -e brdc3540.14n -c 3642325.000,1411093.342,5025370.336 -d 86400 -o pipe.8b -s 4092000
    /home/mkrej/dysk2T/NowyG/SourceCodeZNetu/gps-sdr-sim/gps-sdr-sim -b 8 -e brdc3540.14n -c 3642325.000,1411093.342,5025370.336 -d 86400 -o pipe.8b -s ${fsample} 
    #>> pipe.csv

    #~/GNSS/gps-sdr-sim-master/gps-sdr-sim -b 8 -e brdc3540.14n -c 3642325.000,1411093.342,5025370.336 -d 86400 -o pipe.8b -s 8184000
done


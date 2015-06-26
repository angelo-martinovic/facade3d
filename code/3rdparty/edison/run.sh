start_time=$(date +%s)
./edisonProject run.eds
finish_time=$(date +%s)
echo "Time duration: $((finish_time - start_time)) secs."

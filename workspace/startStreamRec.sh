rm fifo; mkfifo fifo; netcat 192.168.0.32 1234 | cat > fifo

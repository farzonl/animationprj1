open http://localhost:8000/graph.html
python -m SimpleHTTPServer &
httpServer=$!
kill -9 $httpServer
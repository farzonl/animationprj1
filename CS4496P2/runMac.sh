python -m SimpleHTTPServer &
httpServer=$!
open http://localhost:8000/graph.html
kill -9 $httpServer
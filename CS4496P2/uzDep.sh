DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

if [ ! -f "$DIR/Eigen" ]; then
    unzip "$DIR/Eigen.zip" -d $DIR
fi

if [ ! -f "$DIR/nlohmann" ]; then
    unzip "$DIR/nlohmann.zip" -d $DIR
fi

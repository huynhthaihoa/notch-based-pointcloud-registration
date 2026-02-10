# Clear caches
if [ -d "build" ]; then
    rm -r build
fi
if [ -d "CMakeFiles" ]; then
    rm -r CMakeFiles
fi
if [ -f "CMakeCache.txt" ]; then
    rm CMakeCache.txt
fi
if [ -f "Makefile" ]; then
    rm Makefile
fi

if [ ! -d "exec" ]; then
    mkdir exec
fi

# Compile
cmake .
make -j2

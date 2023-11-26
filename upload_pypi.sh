upload () {
    rm -rf ./build
    rm -rf ./dist

    python3 setup.py bdist_wheel sdist

    twine upload dist/*
}

if mypy src; then
    if flake8 src; then
        upload
    fi  
else
    echo "command returned some error"
fi
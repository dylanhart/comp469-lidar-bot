if [ -z `which 2to3` ] ; then
    echo "2to3 (python transpiler) is required"
    exit
fi

if [ -d picoborgrev ] ; then
        echo "library already exists"
else
        wget http://www.piborg.org/downloads/picoborgrev/examples.zip
        unzip examples.zip -d picoborgrev
        rm examples.zip
        2to3 -wno picoborgrev3 picoborgrev
fi
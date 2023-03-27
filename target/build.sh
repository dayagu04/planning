cd ..
# 创建output目录
OutputDir="install/Planning"
if [ -d $OutputDir ]; then rm -rf $OutputDir; fi
mkdir $OutputDir
OutputLibDir="install/Planning/Lib"
if [ -d $OutputLibDir ]; then rm -rf $OutputLibDir; fi
mkdir $OutputLibDir

# 创建build目录
BuildDir="build"
if [ -d $BuildDir ]; then rm -rf $BuildDir; fi
mkdir $BuildDir
cd $BuildDir

cmake ../
if [ "$?" -ne 0 ]; then echo "cmake failed"; exit 1; fi
make
if [ "$?" -ne 0 ]; then echo "make failed"; exit 1; fi
make install
if [ "$?" -ne 0 ]; then echo "make install failed"; exit 1; fi

cd ..

cp target/README.txt install/Interface/
cp CHANGELOG.MD install/Interface/
cp -r doc install/Interface/Doc

cd target/
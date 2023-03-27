DEPLOY_PATH=../deploy

cd $DEPLOY_PATH
chmod +x *.sh

INSTALL_DIR="$PWD/../install"
if [ -d $INSTALL_DIR ]; then rm -rf $INSTALL_DIR; fi
mkdir $INSTALL_DIR

./deploy.sh All ${INSTALL_DIR}

if [ "$?" -ne 0 ]; then echo "deploy failed"; exit 1; fi

echo deploy success
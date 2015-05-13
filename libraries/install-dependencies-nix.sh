#! /bin/bash -e

echo "[spaint] ...Checking dependencies..."
PKGSTOINSTALL=""
for var in "$@"
do
  if [[ ! `dpkg -l | grep -w "ii  $var"` ]];
  then
    PKGSTOINSTALL=$PKGSTOINSTALL" "$var
  else
    echo "[spaint] ...$var... OK"
  fi
done

if [ "$PKGSTOINSTALL" != "" ];
then
  echo "[spaint] ...Installing the following depencencies ${PKGSTOINSTALL}..."
  sudo apt-get install $PKGSTOINSTALL
fi

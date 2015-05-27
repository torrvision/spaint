#! /bin/bash -e

echo "[spaint] ...Checking dependencies..."
PKGSTOINSTALL=""
for var in "$@"
do
  if [[ ! `dpkg -l | grep -w "ii  $var"` ]]
  then
    echo "[spaint] ...$var... MISSING"
    PKGSTOINSTALL="$PKGSTOINSTALL $var"
  else
    echo "[spaint] ...$var... OK"
  fi
done

if [ "$PKGSTOINSTALL" != "" ]
then
  echo "[spaint] ...Installing missing dependencies:$PKGSTOINSTALL ..."
  sudo apt-get install $PKGSTOINSTALL
fi

#! /bin/bash -e

if [ -d ../voicecommander ]
then
  cd apps/spaintgui/resources
  (cat spaint-pre.gram; (cat Labels.txt | perl -pe 's/^(.*)\r/  label \1 \|/' | sed '$d'); echo "  label `tail -n1 Labels.txt`;") > ../../../../voicecommander/resources/spaint.gram
else
  echo "Error: Cannot find the ../voicecommander directory!"
  exit
fi
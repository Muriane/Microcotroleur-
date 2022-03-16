#!/bin/bash

if [ ! $# -eq 2 ] ; then
    echo "usage : $0 <ancien_nom> <nouveau_nom> (sans les extensions)"
    exit 0
fi

ancien=$1
nouveau=$2

for f in `ls $ancien.*` ; do
    extension="${f##*.}"
    base="${f%.*}"
    fnouv="${nouveau}.$extension"
    echo "$f -> $fnouv"
    mv $f $fnouv
done
echo "Terminé"

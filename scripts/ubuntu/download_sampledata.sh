#!/bin/sh

TOPDIR="$(dirname $(realpath "$0"))"/../../ #stella_vslam directory

# create data directory to store downloaded data
mkdir $TOPDIR/dataset
mkdir $TOPDIR/vocab


# download an ORB vocabulary from GitHub
(
    cd ${TOPDIR}/vocab
    curl -sL "https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow" -o orb_vocab.fbow
)

# download a sample dataset from Google Drive
(
    cd $TOPDIR/dataset
    FILE_ID="1d8kADKWBptEqTF7jEVhKatBEdN7g0ikY"
    curl -sc /tmp/cookie "https://drive.google.com/uc?export=download&id=${FILE_ID}" > /dev/null
    CODE="$(awk '/_warning_/ {print $NF}' /tmp/cookie)"
    curl -sLb /tmp/cookie "https://drive.google.com/uc?export=download&confirm=${CODE}&id=${FILE_ID}" -o aist_living_lab_1.zip
    unzip aist_living_lab_1.zip
    
    FILE_ID="1TVf2D2QvMZPHsFoTb7HNxbXclPoFMGLX"
    curl -sc /tmp/cookie "https://drive.google.com/uc?export=download&id=${FILE_ID}" > /dev/null
    CODE="$(awk '/_warning_/ {print $NF}' /tmp/cookie)"
    curl -sLb /tmp/cookie "https://drive.google.com/uc?export=download&confirm=${CODE}&id=${FILE_ID}" -o aist_living_lab_2.zip
    unzip aist_living_lab_2.zip
)

# reference
## https://stella-cv.readthedocs.io/en/latest/simple_tutorial.html#chapter-simple-tutorial

#!/bin/bash
# Copyright (c) 2020 Huawei Device Co., Ltd. All rights reserved.
#
# Compile mpp/sample project, this is the entrance script

# error out on errors
set -e
OUT_DIR="$2"
KERNEL_DIR=$1

function main(){
    mkdir -p $OUT_DIR/../../../libs
    mkdir -p $OUT_DIR/../../../bin
    pushd $KERNEL_DIR
    cp ../../device/allwinner/v3s/sdk_liteos/v3s_liteos_a.patch .
    if [ `grep -c "V3S" Kconfig` -ne '0' ]; then
        echo "patched"
    else
        patch -p 1 < v3s_liteos_a.patch
    fi
    cp ../../device/allwinner/v3s/sdk_liteos/.config .config
    make clean OUTDIR=$OUT_DIR && make rootfs -e -j OUTDIR=$OUT_DIR
    echo $PWD
    cp -rf $OUT_DIR/rootfs*.* $OUT_DIR/../../../
    cp -rf $OUT_DIR/liteos $OUT_DIR/../../../OHOS_Image
    cp -rf $OUT_DIR/liteos.asm $OUT_DIR/../../../OHOS_Image.asm
    cp -rf $OUT_DIR/liteos.map $OUT_DIR/../../../OHOS_Image.map
    mv -f $OUT_DIR/../../../liteos.bin $OUT_DIR/../../../OHOS_Image.bin
    popd
}
main "$@"

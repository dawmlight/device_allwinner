#!/bin/bash
# Copyright (c) 2020 Huawei Device Co., Ltd. All rights reserved.
#
# Compile mpp/sample project, this is the entrance script

# error out on errors
set -e
KERNEL_DIR=$1

function main(){
    pushd $KERNEL_DIR
    cp -f ../../device/allwinner/v3s/sdk_liteos/v3s_liteos_a.patch .
    if [ `grep -c "V3S" platform/Kconfig` -ne '0' ]; then
        echo "patched"
    else
        patch -p 1 < v3s_liteos_a.patch
    fi
    make clean OUTDIR=$OUT_DIR && make rootfs -e -j OUTDIR=$OUT_DIR
    popd
}
main "$@"

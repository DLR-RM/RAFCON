#!/bin/bash

if [ "$#" -ne 1 ]; then
    USER_CHANNEL=ar/stable
else
    USER_CHANNEL=$1
fi

echo "Using user and channel: $USER_CHANNEL"

conan create . $USER_CHANNEL -o python3=False
conan create . $USER_CHANNEL -o python3=True

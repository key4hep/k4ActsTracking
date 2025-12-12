#!/bin/bash
##
## Copyright (c) 2014-2024 Key4hep-Project.
##
## This file is part of Key4hep.
## See https://key4hep.github.io/key4hep-doc/ for further info.
##
## Licensed under the Apache License, Version 2.0 (the "License");
## you may not use this file except in compliance with the License.
## You may obtain a copy of the License at
##
##     http://www.apache.org/licenses/LICENSE-2.0
##
## Unless required by applicable law or agreed to in writing, software
## distributed under the License is distributed on an "AS IS" BASIS,
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
## See the License for the specific language governing permissions and
## limitations under the License.
##

# Download data files with checksum verification using wget and md5sum

set -eu

FILE_LIST=$1
BASE_URL=$2
OUTPUT_DIR=$3

mkdir -p $OUTPUT_DIR
BASE_URL=${BASE_URL%/}

function check_md5() {
    local filename=$1
    local expected_md5=$2

    local actual_md5=$(md5sum $filename | cut -d' ' -f1)
    if [[ "$actual_md5" == "$expected_md5" ]]; then
        return 0
    else
        return 1
    fi
}

function download_file() {
    local filename=$1
    local expected_md5=$2
    local output_file=$OUTPUT_DIR/$filename
    local url=$BASE_URL/$filename

    if wget -q --show-progress -O $output_file $url; then
        if ! check_md5 $output_file $expected_md5; then
            echo "Checksum mismatch for $filename"
            rm -f $output_file
            return 1
        fi
    else
        echo "Failed to download $filename"
    fi
}

while IFS= read -r line || [[ -n "$line" ]]; do
    # Skip empty lines and comments
    if [[ -z "$line" || "$line" =~ ^[[:space:]]*# ]]; then
        continue
    fi

    read -r filename expected_md5 <<< "$line"

    if [ ! -f ${OUTPUT_DIR}/${filename} ]; then
        download_file "$filename" "$expected_md5"
    else
        if ! check_md5 "$OUTPUT_DIR/$filename" "$expected_md5"; then
            download_file "$filename" "$expected_md5"
        fi
    fi
done < $FILE_LIST


echo "All files downloaded successfully!"
exit 0

#!/usr/bin/env bash
#
# Copyright (c) 2010 - 2024, Fraunhofer-Gesellschaft zur Foerderung der angewandten Forschung e.V.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# We kindly request you to use one or more of the following phrases to refer to
# foxBMS in your hardware, software, documentation or advertising materials:
#
# - "This product uses parts of foxBMS®"
# - "This product includes parts of foxBMS®"
# - "This product is derived from foxBMS®"

set -e

SCRIPTDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

ENV_NAME="2024-08-pale-fox"
# MacOS
if [ "$(uname)" == "Darwin" ]; then
    echo "MacOS is currently not supported."
    exit 1
# Linux
elif [ "$(printf "%s" "$(uname -s)" | cut -c 1-5)" == "Linux" ]; then
    pushd "$SCRIPTDIR" > /dev/null
    FOXBMS_PYTHON_ENV_DIRECTORY_USER="$HOME/foxbms-envs/${ENV_NAME}"
    FOXBMS_PYTHON_ENV_DIRECTORY_ROOT="/opt/foxbms-envs/${ENV_NAME}"

    FOXBMS_PYTHON_ENV_DIRECTORY="$FOXBMS_PYTHON_ENV_DIRECTORY_USER"

    # Prefer the user installation
    if [ ! -d "$FOXBMS_PYTHON_ENV_DIRECTORY_USER" ]; then
        FOXBMS_PYTHON_ENV_DIRECTORY="$FOXBMS_PYTHON_ENV_DIRECTORY_ROOT"
    fi

    if [ ! -d "$FOXBMS_PYTHON_ENV_DIRECTORY" ]; then
        echo "'$FOXBMS_PYTHON_ENV_DIRECTORY_USER' and"
        echo "'$FOXBMS_PYTHON_ENV_DIRECTORY_ROOT' do not exist."
        echo "One of both must be available. See Installation instructions in"
        echo "'$SCRIPTDIR/INSTALL.md'"
        exit 1
    fi

    # shellcheck source=/dev/null
    source "$FOXBMS_PYTHON_ENV_DIRECTORY/bin/activate"

    # Check if Python executable exists
    if ! command -v python &> /dev/null
    then
        echo "Could not find python executable"
        exit 1
    fi

    python "${SCRIPTDIR}/fox.py" "$@" || rc="$?"
    deactivate
    popd > /dev/null
    exit $((rc))
# Windows
elif [ "$(printf "%s" "$(uname -s)" | cut -c 1-9)" == "CYGWIN_NT" ]; then
    echo "Cygwin is not supported."
    exit 1
elif [ "$(printf "%s" "$(uname -s)" | cut -c 1-10)" == "MINGW32_NT" ]; then
    echo "32bit Windows is not supported."
    exit 1
elif [ "$(printf "%s" "$(uname -s)" | cut -c 1-10)" == "MINGW64_NT" ] || [ "$(printf "%s" "$(uname -s)" | cut -c 1-7)" == "MSYS_NT" ] ; then
    pushd "$SCRIPTDIR" > /dev/null
    UNIX_USERPROFILE="${USERPROFILE//\\//}" # replace backslashes with forward slashes
    UNIX_USERPROFILE="${UNIX_USERPROFILE//\:/}" # remove drive colon
    FOXBMS_PYTHON_ENV_DIRECTORY_USER="/${UNIX_USERPROFILE}/foxbms-envs/${ENV_NAME}"
    FOXBMS_PYTHON_ENV_DIRECTORY_ROOT="/C/foxbms-envs/${ENV_NAME}"

    FOXBMS_PYTHON_ENV_DIRECTORY="$FOXBMS_PYTHON_ENV_DIRECTORY_USER"

    # Prefer the user installation
    if [ ! -d "$FOXBMS_PYTHON_ENV_DIRECTORY_USER" ]; then
        FOXBMS_PYTHON_ENV_DIRECTORY="$FOXBMS_PYTHON_ENV_DIRECTORY_ROOT"
    fi

    if [ ! -d "$FOXBMS_PYTHON_ENV_DIRECTORY" ]; then
        echo "'$FOXBMS_PYTHON_ENV_DIRECTORY_USER' and"
        echo "'$FOXBMS_PYTHON_ENV_DIRECTORY_ROOT' do not exist."
        echo "One of both must be available. See Installation instructions in"
        echo "'$SCRIPTDIR/INSTALL.md'"
        exit 1
    fi

    # shellcheck source=/dev/null
    source "$FOXBMS_PYTHON_ENV_DIRECTORY/Scripts/activate"

    # Check if Python executable exists
    if ! command -v python &> /dev/null
    then
        echo "Could not find python executable"
        exit 1
    fi

    python "${SCRIPTDIR}/fox.py" "$@" || rc="$?"
    deactivate
    popd > /dev/null
    exit $((rc))
fi

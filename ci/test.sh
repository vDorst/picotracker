#!/bin/bash

set -euo pipefail

cargo test --target x86_64-unknown-linux-gnu -p modbus_master
cargo test --target x86_64-unknown-linux-gnu -p gpslib
cargo test --target x86_64-unknown-linux-gnu -p envconst

#!/bin/sh

# Remove -Werror from all makefiles
makefiles="$(find . -type f -name Makefile)
    $(find . -type f -name Kbuild)"
for i in $makefiles; do
    sed -i 's/-Werror-/-W/g' "$i"
    sed -i 's/-Werror//g' "$i"
done

ROOT=`git rev-parse --show-toplevel`

# Clang format
FORMAT="xargs clang-format -i --style=LLVM"
FIND_PATTERN="-iname *.hpp -o -iname *.cpp -o -iname *.h"

find "$ROOT/src" $FIND_PATTERN | $FORMAT
find "$ROOT/include" $FIND_PATTERN | $FORMAT
find "$ROOT/test" $FIND_PATTERN | $FORMAT


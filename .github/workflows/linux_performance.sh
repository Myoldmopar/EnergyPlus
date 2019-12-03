#!/usr/bin/env bash

# make a results dir for later archiving
mkdir results

# checkout the repo into a "repo" subdirectory and go in there
git clone "https://github.com/${GITHUB_REPOSITORY}" repo
cd repo

# checkout the base branch, build, and run tests
git checkout "${GITHUB_SHA}"  # develop eventually
mkdir build && cd build
cmake -DBUILD_TESTING=ON -DBUILD_PERFORMANCE_TESTS:BOOL=ON -DVALGRIND_ANALYZE_PERFORMANCE_TESTS:BOOL=ON ..
make -j 3
ctest -j 2 -R "performance.*"

# move into the performance_tests folder to gather results
cd performance_tests
python3 ../../scripts/dev/gather_performance_results.py

# now move to the original folder and store the results
cd ../..
cp repo/build/performance_tests/performance.json ./results/performance_base.json

# delete the repo folder to start over
rm -rf repo/

# checkout the repo into a "repo" subdirectory and go in there
git clone "https://github.com/${GITHUB_REPOSITORY}" repo
cd repo

# checkout this commit branch, build, and run tests -- CCACHE should help here in most cases!
git checkout "${GITHUB_SHA}"
mkdir build && cd build
cmake -DBUILD_TESTING=ON -DBUILD_PERFORMANCE_TESTS:BOOL=ON -DVALGRIND_ANALYZE_PERFORMANCE_TESTS:BOOL=ON ..
make -j 3
ctest -j 2 -R "performance.*"

# move into the performance_tests folder to gather results
cd performance_tests
python3 ../../scripts/dev/gather_performance_results.py

# now move to the original folder and store the results
cd ../..
cp repo/build/performance_tests/performance.json ./results/performance_mod.json


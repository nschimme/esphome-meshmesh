#!/bin/bash

function update_component() {
  cd -
  declare -a arr=("esphome" "meshmesh" "meshmesh_direct" "network" "socket")
  for item in "${arr[@]}"; do
    rsync -ar ../esphome-pub/esphome/components/${item}/ components/${item} --exclude __pycache__ --delete
  done
  git commit -a -m "$1"
  cd -
}


LAST_COMMIT=$(cat .last_commit)
echo "Last commit:"
echo $LAST_COMMIT

cd ../esphome-pub

NEW_COMMITS=$(git rev-list $LAST_COMMIT..HEAD | tac)

if [ -z "$NEW_COMMITS" ]; then
  echo "No new commits"
  exit 0
fi

git stash
for commit in $NEW_COMMITS; do
  echo "Commit:"
  echo $commit
  git checkout $commit
  CURRENT_MESSAGE=$(git log -1 --pretty=%B)
  update_component "$CURRENT_MESSAGE" 
done
git checkout mm_dev
git stash pop

cd -

echo $LAST_COMMIT > .last_commit


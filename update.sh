#!/bin/bash

MY_EMAIL="stefano.pagnottelli@siralab.com"

function update_component() {
  cd -
  declare -a arr=("esphome" "meshmesh" "meshmesh_direct" "network" "socket", "audio")
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

NEW_COMMITS=$(git rev-list --author=${MY_EMAIL} $LAST_COMMIT..HEAD | tac)

if [ -z "$NEW_COMMITS" ]; then
  echo "No new commits"
  exit 0
fi

git stash
for commit in $NEW_COMMITS; do
  echo "Commit: $commit"
  git checkout $commit
  CURRENT_MESSAGE=$(git log -1 --pretty=%B)
  CURRENT_EMAIL=$(git log -1 --pretty=%ae)

  if [ "$CURRENT_EMAIL" != "${MY_EMAIL}" ]; then
    echo "Commit not made by Stefano Pagnottelli, skipping"
    continue
  fi

  LAST_COMMIT=$commit
  update_component "$CURRENT_MESSAGE" 
  
done
git checkout mm_dev
git stash pop

cd -

echo $LAST_COMMIT > .last_commit


#!/bin/bash
set -e

REPO_DIR="$HOME/src"
COMMIT_MSG="${1:-Update src repository}"

cd "$REPO_DIR"

echo "Repository: $REPO_DIR"
git status --short

if [ -z "$(git status --porcelain)" ]; then
  echo "Nessuna modifica da committare."
  exit 0
fi

git add .
git commit -m "$COMMIT_MSG"
git push

echo "Push completato."

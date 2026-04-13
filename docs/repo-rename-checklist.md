# Repository Rename Checklist (Tap-O-Matic-DDV -> Dub-Machine)

## 1) Rename on GitHub
1. Open repository Settings.
2. Change repository name to `Dub-Machine` (or your preferred final name).
3. Save changes.

GitHub will create redirects from old URLs, but update local remotes anyway.

## 2) Update local git remote
Run in your local clone:

```bash
git remote -v
git remote set-url origin git@github.com:<owner>/Dub-Machine.git
# or HTTPS:
# git remote set-url origin https://github.com/<owner>/Dub-Machine.git
git remote -v
```

## 3) Validate CI and badges
1. Open Actions and run a firmware build manually.
2. Check README badges/links and update old repo URLs.
3. Confirm artifact names and workflow status links resolve.

## 4) Optional follow-up cleanup
1. Rename any remaining ToM/Tap-specific binary filenames under `binaries/` if desired.
2. Update issue templates, release notes templates, and project board links.
3. Notify collaborators to run `git remote set-url` in existing clones.

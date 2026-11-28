#!/usr/bin/env bash
# FM Pilot: one-time setup of a local FM Pilot workspace in WSL2 Ubuntu 24.04 on Frankie's Windows PC.
#
# Run from the Ubuntu (WSL) prompt:
#   curl -fsSL https://raw.githubusercontent.com/pmalekian/fmpilot/fm-dev-sync/tools/fm/wsl_setup.sh | bash
# or, from an existing clone:  bash tools/fm/wsl_setup.sh
#
# Safe to run again: every step checks what is already done. It asks for your Ubuntu password once (sudo).
# Options (env vars): FM_DIR (default ~/fmpilot), FM_BRANCH (default fm-dev-sync), COMMA_IP (default 192.168.1.246),
# FM_SKIP_BUILD=1 to skip the scons build.
set -euo pipefail

FM_DIR="${FM_DIR:-$HOME/fmpilot}"
FM_BRANCH="${FM_BRANCH:-fm-dev-sync}"
COMMA_IP="${COMMA_IP:-192.168.1.246}"
REPO_URL="https://github.com/pmalekian/fmpilot.git"
LOG="$HOME/fm_setup.log"

step() { echo; echo "=== [$(date +%H:%M:%S)] $*"; }
ok()   { echo "  [ok] $*"; }
warn() { echo "  [!!] $*"; }

main() {
exec > >(tee -a "$LOG") 2>&1
step "Checks"
if ! grep -qi microsoft /proc/version; then warn "This doesn't look like WSL. Continuing anyway."; fi
if [[ "$(pwd)" == /mnt/* ]]; then cd "$HOME"; fi   # never build on the Windows drive (slow)
case "$FM_DIR" in /mnt/*) echo "FM_DIR must be inside the Linux filesystem (e.g. ~/fmpilot), not $FM_DIR"; exit 1;; esac
ok "Ubuntu $(. /etc/os-release && echo "$VERSION_ID"), $(nproc) cores, $(free -g | awk '/Mem:/{print $2}') GB RAM visible to WSL"

step "System packages (asks for your Ubuntu password)"
sudo apt-get update -qq
sudo apt-get install -y -qq git git-lfs curl ca-certificates build-essential openssh-client gh xvfb zstd \
  libcurl4-openssl-dev locales xclip >/dev/null
git lfs install --skip-repo >/dev/null
ok "git, git-lfs, gh, ssh, xvfb installed"

step "GPU"
if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
  ok "$(nvidia-smi -L | head -1)"
else
  warn "nvidia-smi not working in WSL. Update the NVIDIA driver on Windows (do NOT install a driver inside Ubuntu)."
fi

step "Clone $FM_BRANCH into $FM_DIR"
export GIT_LFS_SKIP_SMUDGE=1   # large files come from `git lfs pull` below, so one failed download can't break the clone
if [[ -d "$FM_DIR/.git" ]]; then
  ok "clone exists, fetching"
  git -C "$FM_DIR" fetch -q origin "$FM_BRANCH"
  git -C "$FM_DIR" checkout "$FM_BRANCH"
  git -C "$FM_DIR" pull --ff-only origin "$FM_BRANCH" || warn "local changes or divergence; left as is"
else
  git clone -q --filter=blob:none -b "$FM_BRANCH" "$REPO_URL" "$FM_DIR"
fi
cd "$FM_DIR"
git submodule update -q --init --recursive --jobs 4
unset GIT_LFS_SKIP_SMUDGE
git lfs pull 2>&1 | tail -3 || true
missing_lfs=$(git lfs ls-files 2>/dev/null | grep -c " - " || true)
if [[ "$missing_lfs" -gt 0 ]]; then warn "$missing_lfs LFS files (models, images) not downloaded; rerun later: cd $FM_DIR && git lfs pull"; fi
git config user.name "Frankie Malekian"
git config user.email "107580415+pmalekian@users.noreply.github.com"
ok "at $(git rev-parse --short HEAD) on $(git rev-parse --abbrev-ref HEAD)"

step "Python environment (uv)"
bash tools/setup_dependencies.sh
export PATH="$HOME/.local/bin:$PATH"
# shellcheck disable=SC1091
source .venv/bin/activate
uv pip install -q pytest pytest-timeout
ok "venv at $FM_DIR/.venv ($(python --version))"

if [[ "${FM_SKIP_BUILD:-0}" != "1" ]]; then
  step "Build (scons -j$(nproc)); first build takes a while"
  if scons -j"$(nproc)" -k > "$HOME/fm_build.log" 2>&1; then ok "build passed"; else warn "build had errors; see ~/fm_build.log"; fi
fi

step "Quick check: FM log tool tests"
if (cd tools/fm && python -m pytest -q tests > "$HOME/fm_tools_tests.log" 2>&1); then
  ok "tools/fm tests passed"
else
  warn "tools/fm tests failed; see ~/fm_tools_tests.log"
fi

step "SSH to the comma ($COMMA_IP)"
mkdir -p "$HOME/.ssh" && chmod 700 "$HOME/.ssh"
WIN_HOME="$(wslpath "$(cmd.exe /c 'echo %USERPROFILE%' 2>/dev/null </dev/null | tr -d '\r')" 2>/dev/null || true)"
for k in id_ed25519 id_rsa; do
  if [[ ! -f "$HOME/.ssh/$k" && -n "$WIN_HOME" && -f "$WIN_HOME/.ssh/$k" ]]; then
    cp "$WIN_HOME/.ssh/$k" "$HOME/.ssh/$k"; chmod 600 "$HOME/.ssh/$k"
    [[ -f "$WIN_HOME/.ssh/$k.pub" ]] && cp "$WIN_HOME/.ssh/$k.pub" "$HOME/.ssh/$k.pub"
    ok "copied your Windows SSH key $k"
  fi
done
if ! grep -q "^Host comma$" "$HOME/.ssh/config" 2>/dev/null; then
  printf 'Host comma\n  HostName %s\n  User comma\n  StrictHostKeyChecking accept-new\n  ConnectTimeout 5\n' "$COMMA_IP" >> "$HOME/.ssh/config"
  chmod 600 "$HOME/.ssh/config"
  ok "added 'ssh comma' shortcut"
fi
if ssh -o BatchMode=yes comma true 2>/dev/null; then ok "ssh comma works"
else warn "can't reach the comma right now (car off/asleep, or key not set up). Retry later: ssh comma"; fi

step "Claude Code"
if ! command -v claude >/dev/null 2>&1 && [[ ! -x "$HOME/.local/bin/claude" ]]; then
  curl -fsSL https://claude.ai/install.sh | bash || warn "Claude Code install failed; retry: curl -fsSL https://claude.ai/install.sh | bash"
fi
touch "$HOME/.bashrc"
grep -q 'HOME/.local/bin' "$HOME/.bashrc" || echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
grep -q 'fmpilot/.venv' "$HOME/.bashrc" || echo "[ -f $FM_DIR/.venv/bin/activate ] && source $FM_DIR/.venv/bin/activate" >> "$HOME/.bashrc"
ok "$("$HOME/.local/bin/claude" --version 2>/dev/null || claude --version 2>/dev/null || echo 'installed')"

step "Done"
cat <<EOF
Log: $LOG
Next, at this Ubuntu prompt:
  1. gh auth login        (GitHub.com > HTTPS > login with a web browser; lets local sessions push)
  2. cd $FM_DIR && claude (sign in once, then type /remote-control to connect from your phone)
Later, to start a phone-controllable session directly:  cd $FM_DIR && claude --remote-control
EOF
}

# Wrapped in main so `curl | bash` reads the whole script before running anything.
main "$@" </dev/null

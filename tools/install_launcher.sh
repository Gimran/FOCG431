#!/usr/bin/env bash
# Ставит ярлык SimpleFOC CAN Studio: в меню приложений и на рабочий стол.
# Запуск: bash tools/install_launcher.sh [путь_к_can_gui.py]
# Без аргумента берётся can_gui.py рядом с этим скриптом (уровнем выше).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GUI="${1:-$HERE/../can_gui.py}"
GUI="$(cd "$(dirname "$GUI")" && pwd)/$(basename "$GUI")"
[ -f "$GUI" ] || { echo "не найден can_gui.py: $GUI" >&2; exit 1; }

WORKDIR="$(dirname "$GUI")"
ICON="$HERE/icon.png"
# иконку кладём рядом с GUI, чтобы ярлык не зависел от папки репозитория
[ -f "$ICON" ] && cp -f "$ICON" "$WORKDIR/icon.png"
ICON="$WORKDIR/icon.png"

APPS="$HOME/.local/share/applications"
DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
FILE="simplefoc-can-studio.desktop"
mkdir -p "$APPS"

cat > "$APPS/$FILE" <<EOF
[Desktop Entry]
Type=Application
Version=1.0
Name=SimpleFOC CAN Studio
Comment=Управление сервоприводом FOCG431 по шине CAN
Exec=python3 "$GUI"
Path=$WORKDIR
Icon=$ICON
Terminal=false
Categories=Development;Electronics;Engineering;
Keywords=CAN;SimpleFOC;STM32;servo;
StartupNotify=true
EOF

chmod +x "$APPS/$FILE"
update-desktop-database "$APPS" 2>/dev/null || true
echo "меню приложений:  $APPS/$FILE"

# Копии для запуска мышью. GNOME запускает такой ярлык, только пометив доверенным.
for dir in "$DESKTOP_DIR" "$WORKDIR"; do
    [ -d "$dir" ] || continue
    [ "$dir" = "$APPS" ] && continue
    cp -f "$APPS/$FILE" "$dir/$FILE"
    chmod +x "$dir/$FILE"
    gio set "$dir/$FILE" metadata::trusted true 2>/dev/null || true
    echo "ярлык:            $dir/$FILE"
done

echo
echo "Готово. Ярлык появится в меню приложений (поиск: SimpleFOC)."
echo "Если значок на рабочем столе не запускается - правый клик -> Allow Launching."

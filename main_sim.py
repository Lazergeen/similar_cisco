import sys
from PyQt6.QtCore import Qt, QPointF, QVariantAnimation
from PyQt6.QtGui import QAction, QIcon, QPainter, QTextCursor, QKeySequence, QFont
from PyQt6.QtWidgets import (
    QApplication,
    QDialog,
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsRectItem,
    QGraphicsScene,
    QGraphicsTextItem,
    QGraphicsView,
    QMainWindow,
    QPlainTextEdit,
    QToolBar,
    QVBoxLayout,
    QMessageBox,
)

# ---------------------- GUI SECTION ----------------------

class DraggableDeviceItem(QGraphicsItem):
    """Base class for draggable network device items."""

    def __init__(self, label: str, color: Qt.GlobalColor, rect_size: float = 60.0):
        super().__init__()
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges, True)
        self.label = label
        self.rect_size = rect_size
        self.color = color
        self.on_double_click = None  # type: ignore[assignment]
        self._init_children()

    def _init_children(self):
        # Simple body shape
        if self.label.lower().startswith("router"):
            body = QGraphicsEllipseItem(0, 0, self.rect_size, self.rect_size, self)
        else:
            body = QGraphicsRectItem(0, 0, self.rect_size, self.rect_size, self)
        body.setBrush(self.color)
        body.setPen(Qt.GlobalColor.black)
        body.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, False)
        # Text label
        text = QGraphicsTextItem(self.label, self)
        text.setDefaultTextColor(Qt.GlobalColor.white)
        # Center text
        b = text.boundingRect()
        text.setPos((self.rect_size - b.width()) / 2, (self.rect_size - b.height()) / 2)

    def mouseDoubleClickEvent(self, event):
        if callable(self.on_double_click):
            self.on_double_click(self)
        else:
            super().mouseDoubleClickEvent(event)

    def boundingRect(self):
        return self.childrenBoundingRect()

    def paint(self, painter: QPainter, option, widget=None):
        # All painting is handled by child items
        pass


# ---------------------- CLI CORE (reused for per-device) ----------------------

from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field

from prompt_toolkit import PromptSession
from prompt_toolkit.completion import Completer, Completion
from prompt_toolkit.formatted_text import HTML
from prompt_toolkit.history import InMemoryHistory


@dataclass
class InterfaceConfig:
    name: str
    ip_address: Optional[Tuple[str, str]] = None  # (ip, mask)


@dataclass
class DeviceConfig:
    hostname: str = "ciskosim"
    interfaces: Dict[str, InterfaceConfig] = field(default_factory=dict)

    def get_running_config_lines(self) -> List[str]:
        lines: List[str] = [f"hostname {self.hostname}"]
        for if_name, iface in self.interfaces.items():
            lines.append(f"interface {if_name}")
            if iface.ip_address:
                ip, mask = iface.ip_address
                lines.append(f" ip address {ip} {mask}")
            lines.append(" exit")
        return lines


class Mode:
    USER = ">"
    ENABLE = "#"
    CONFIG = "(config)#"
    IFACE = "(config-if)#"


class CiscoLikeCompleter(Completer):
    def __init__(self, mode: str, device: DeviceConfig, current_interface: Optional[str]):
        self.mode = mode
        self.device = device
        self.current_interface = current_interface

    def get_completions(self, document, complete_event):
        text = document.text_before_cursor.strip()
        parts = text.split()

        def complete_word(candidates: List[str]):
            word = parts[-1] if parts else ""
            for c in candidates:
                if c.startswith(word):
                    yield Completion(c, start_position=-len(word))

        if self.mode == Mode.USER:
            if not parts:
                for c in ["enable", "show"]:
                    yield Completion(c, start_position=0)
            elif parts[0] == "show":
                yield from complete_word(["running-config", "ip"])
            elif parts[0] == "ip":
                pass
            else:
                yield from complete_word(["enable", "show"])        
        elif self.mode == Mode.ENABLE:
            if not parts:
                for c in ["disable", "configure", "show"]:
                    yield Completion(c, start_position=0)
            elif parts[0] == "configure":
                yield from complete_word(["terminal"])
            elif parts[0] == "show":
                yield from complete_word(["running-config", "ip"])
            elif parts[0] == "ip":
                pass
            else:
                yield from complete_word(["disable", "configure", "show"])        
        elif self.mode == Mode.CONFIG:
            if not parts:
                for c in ["exit", "interface", "hostname"]:
                    yield Completion(c, start_position=0)
            elif parts[0] == "interface":
                candidates = list(self.device.interfaces.keys()) or [
                    "GigabitEthernet0/0",
                    "GigabitEthernet0/1",
                    "Loopback0",
                ]
                yield from complete_word(candidates)
            elif parts[0] == "hostname":
                pass
            else:
                yield from complete_word(["exit", "interface", "hostname"])        
        elif self.mode == Mode.IFACE:
            if not parts:
                for c in ["exit", "ip", "shutdown", "no"]:
                    yield Completion(c, start_position=0)
            elif parts[0] == "ip":
                yield from complete_word(["address"])
            elif parts[0] == "no":
                yield from complete_word(["shutdown"])
            else:
                yield from complete_word(["exit", "ip", "shutdown", "no"])        


class DeviceShell:
    def __init__(self, device_config: DeviceConfig):
        self.device = device_config
        self.mode = Mode.USER
        self.current_interface: Optional[str] = None

    def prompt(self) -> str:
        if self.mode == Mode.USER:
            return f"{self.device.hostname}> "
        if self.mode == Mode.ENABLE:
            return f"{self.device.hostname}# "
        if self.mode == Mode.CONFIG:
            return f"{self.device.hostname}(config)# "
        return f"{self.device.hostname}(config-if)# "

    def handle(self, line: str) -> List[str]:
        out: List[str] = []
        parts = line.split()
        if not parts:
            return out
        cmd = parts[0]
        args = parts[1:]
        def println(s: str):
            out.append(s)

        if self.mode == Mode.USER:
            if cmd == "enable":
                self.mode = Mode.ENABLE
            elif cmd == "show":
                out.extend(self._cmd_show(args))
            elif cmd in ("quit", "exit"):
                println("Use Ctrl+W or close window to exit terminal.")
            else:
                println("% Invalid input detected at '^' marker.")
        elif self.mode == Mode.ENABLE:
            if cmd == "disable":
                self.mode = Mode.USER
            elif cmd == "configure" and args[:1] == ["terminal"]:
                self.mode = Mode.CONFIG
            elif cmd == "show":
                out.extend(self._cmd_show(args))
            else:
                println("% Invalid input detected at '^' marker.")
        elif self.mode == Mode.CONFIG:
            if cmd == "exit":
                self.mode = Mode.ENABLE
            elif cmd == "hostname" and args:
                self.device.hostname = args[0]
            elif cmd == "interface" and args:
                if_name = args[0]
                if if_name not in self.device.interfaces:
                    self.device.interfaces[if_name] = InterfaceConfig(name=if_name)
                self.current_interface = if_name
                self.mode = Mode.IFACE
            else:
                println("% Invalid input detected at '^' marker.")
        elif self.mode == Mode.IFACE:
            if cmd == "exit":
                self.current_interface = None
                self.mode = Mode.CONFIG
            elif cmd == "ip" and args[:1] == ["address"] and len(args) >= 3:
                ip, mask = args[1], args[2]
                assert self.current_interface is not None
                self.device.interfaces[self.current_interface].ip_address = (ip, mask)
            elif cmd == "shutdown":
                println("Interface administratively down (not persisted in this demo)")
            elif cmd == "no" and args[:1] == ["shutdown"]:
                println("Interface up (not persisted in this demo)")
            else:
                println("% Invalid input detected at '^' marker.")
        return out

    def _cmd_show(self, args: List[str]) -> List[str]:
        out: List[str] = []
        def println(s: str):
            out.append(s)
        if not args:
            println("% Incomplete command.")
            return out
        if args[0] == "running-config":
            out.extend(self.device.get_running_config_lines())
        elif args[0] == "ip":
            if len(args) >= 3 and args[1] == "interface" and args[2] == "brief":
                println("Interface\tIP-Address\tMask")
                for if_name, iface in self.device.interfaces.items():
                    ip_str = iface.ip_address[0] if iface.ip_address else "unassigned"
                    mask_str = iface.ip_address[1] if iface.ip_address else "-"
                    println(f"{if_name}\t{ip_str}\t{mask_str}")
            else:
                println("% Incomplete command.")
        else:
            println("% Invalid input detected at '^' marker.")
        return out


class TerminalDialog(QDialog):
    def __init__(self, device_config: DeviceConfig, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Terminal - {device_config.hostname}")
        self.resize(800, 500)
        self.shell = DeviceShell(device_config)

        # Single-pane terminal
        self.output = QPlainTextEdit(self)
        self.output.setUndoRedoEnabled(False)
        self.output.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self.output.installEventFilter(self)

        # Styling like a typical terminal
        mono = QFont("DejaVu Sans Mono")
        mono.setStyleHint(QFont.StyleHint.Monospace)
        mono.setFixedPitch(True)
        mono.setPointSize(11)
        self.output.setFont(mono)
        self.output.setStyleSheet("QPlainTextEdit { background: #111; color: #e5e5e5; }")

        layout = QVBoxLayout()
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(self.output)
        self.setLayout(layout)

        # History support
        self._history: list[str] = []
        self._hist_index: int = -1
        self._prompt_pos: int = 0

        self._print(self.shell.prompt())
        self._prompt_pos = self._cursor_pos()

    # ------------- helpers -------------
    def _cursor(self):
        return self.output.textCursor()

    def _set_cursor(self, cursor: QTextCursor):
        self.output.setTextCursor(cursor)

    def _cursor_pos(self) -> int:
        return self._cursor().position()

    def _move_cursor_end(self):
        self.output.moveCursor(QTextCursor.MoveOperation.End)

    def _ensure_at_input(self):
        if self._cursor_pos() < self._prompt_pos:
            cursor = self._cursor()
            cursor.setPosition(self._prompt_pos)
            self._set_cursor(cursor)

    def _print(self, text: str):
        self.output.moveCursor(QTextCursor.MoveOperation.End)
        self.output.insertPlainText(text)
        self.output.ensureCursorVisible()

    def _println(self, text: str):
        self._print(text + "\n")

    def _current_input(self) -> str:
        doc = self.output.document()
        return doc.toPlainText()[self._prompt_pos:]

    def _replace_current_input(self, text: str):
        self._move_cursor_end()
        cursor = self._cursor()
        cursor.setPosition(self._prompt_pos, QTextCursor.MoveMode.KeepAnchor)
        cursor.movePosition(QTextCursor.MoveOperation.End, QTextCursor.MoveMode.KeepAnchor)
        cursor.removeSelectedText()
        self._set_cursor(cursor)
        self._print(text)

    def _new_prompt(self):
        self._print(self.shell.prompt())
        self._prompt_pos = self._cursor_pos()

    # ------------- event handling -------------
    def eventFilter(self, obj, event):
        if obj is self.output and event.type() == event.Type.KeyPress:
            key = event.key()
            mods = event.modifiers()
            # Prevent editing before prompt
            if key in (Qt.Key.Key_Backspace,):
                if self._cursor_pos() <= self._prompt_pos:
                    return True
            if key in (Qt.Key.Key_Left,):
                if self._cursor_pos() <= self._prompt_pos:
                    return True
            if key == Qt.Key.Key_Home:
                cursor = self._cursor()
                cursor.setPosition(self._prompt_pos)
                self._set_cursor(cursor)
                return True
            if key == Qt.Key.Key_End:
                self._move_cursor_end()
                return True
            if key == Qt.Key.Key_L and (mods & Qt.KeyboardModifier.ControlModifier):
                self.output.clear()
                self._new_prompt()
                return True
            if key == Qt.Key.Key_Up:
                if self._history:
                    if self._hist_index == -1:
                        self._hist_index = len(self._history) - 1
                    else:
                        self._hist_index = max(0, self._hist_index - 1)
                    self._replace_current_input(self._history[self._hist_index])
                return True
            if key == Qt.Key.Key_Down:
                if self._history:
                    if self._hist_index == -1:
                        return True
                    self._hist_index += 1
                    if self._hist_index >= len(self._history):
                        self._hist_index = -1
                        self._replace_current_input("")
                    else:
                        self._replace_current_input(self._history[self._hist_index])
                return True
            if key in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
                cmd = self._current_input()
                self._print("\n")
                if cmd:
                    self._history.append(cmd)
                    self._hist_index = -1
                # handle
                out_lines = self.shell.handle(cmd)
                for l in out_lines:
                    self._println(l)
                self._new_prompt()
                return True
            # For other printable characters, ensure we stay at input region
            self._ensure_at_input()
        return super().eventFilter(obj, event)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CiskoSimilar - Network Simulator")
        self.resize(1000, 700)

        # Storage for device positions: {item_id: (x, y, label)}
        self.device_positions: dict[int, dict] = {}
        self._next_id = 1

        # Device registry mapping
        # id -> {label, ip}
        self.device_registry: dict[int, dict] = {}
        # id <-> item mapping
        self.id_to_item: dict[int, DraggableDeviceItem] = {}
        self.item_to_id: dict[DraggableDeviceItem, int] = {}
        # Per-device configs
        self.device_configs: dict[int, DeviceConfig] = {}

        # Scene/View workspace
        self.scene = QGraphicsScene(self)
        self.scene.setSceneRect(0, 0, 2000, 1200)
        self.view = QGraphicsView(self.scene, self)
        self.view.setRenderHints(QPainter.RenderHint.Antialiasing)
        self.view.setDragMode(QGraphicsView.DragMode.RubberBandDrag)
        self.setCentralWidget(self.view)

        # Toolbar
        self._build_toolbar()

    def _build_toolbar(self):
        toolbar = QToolBar("Devices", self)
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        add_router_action = QAction(QIcon(), "Add Router", self)
        add_router_action.triggered.connect(self.add_router)
        toolbar.addAction(add_router_action)

        add_switch_action = QAction(QIcon(), "Add Switch", self)
        add_switch_action.triggered.connect(self.add_switch)
        toolbar.addAction(add_switch_action)

        send_packet_action = QAction(QIcon(), "Send Packet", self)
        send_packet_action.triggered.connect(self.send_packet)
        toolbar.addAction(send_packet_action)

    def _assign_ip_for_id(self, device_id: int) -> str:
        # simple scheme 10.0.0.X
        return f"10.0.0.{device_id}"

    def _add_device(self, label_prefix: str, at_pos: QPointF | None = None, color: Qt.GlobalColor = Qt.GlobalColor.darkCyan):
        device_id = self._next_id
        self._next_id += 1

        label = f"{label_prefix} {device_id}"
        item = DraggableDeviceItem(label=label, color=color)
        self.scene.addItem(item)

        start_pos = at_pos if at_pos is not None else self.view.mapToScene(50, 50)
        item.setPos(start_pos)

        # Track position
        self.device_positions[device_id] = {"label": label, "x": float(start_pos.x()), "y": float(start_pos.y())}

        # Registry and mapping
        ip_addr = self._assign_ip_for_id(device_id)
        self.device_registry[device_id] = {"label": label, "ip": ip_addr}
        self.id_to_item[device_id] = item
        self.item_to_id[item] = device_id

        # Per-device config
        cfg = DeviceConfig(hostname=label)
        self.device_configs[device_id] = cfg

        # Double-click handler to open terminal
        def open_term(_):
            self.open_terminal_for_device(device_id)
        item.on_double_click = open_term

        # Handle item move updates by intercepting its itemChange
        self._attach_position_tracking(item, device_id)

    def _attach_position_tracking(self, item: DraggableDeviceItem, device_id: int):
        original_item_change = item.itemChange

        def item_change_proxy(change, value):
            if change == QGraphicsItem.GraphicsItemChange.ItemPositionChange:
                new_pos: QPointF = value
                self.device_positions[device_id]["x"] = float(new_pos.x())
                self.device_positions[device_id]["y"] = float(new_pos.y())
            return original_item_change(change, value)

        item.itemChange = item_change_proxy  # type: ignore[method-assign]

    def add_router(self):
        self._add_device("Router", color=Qt.GlobalColor.blue)

    def add_switch(self):
        self._add_device("Switch", color=Qt.GlobalColor.darkGreen)

    def open_terminal_for_device(self, device_id: int):
        cfg = self.device_configs[device_id]
        dlg = TerminalDialog(cfg, self)
        dlg.setWindowTitle(f"Terminal - {self.device_registry[device_id]['label']}")
        dlg.exec()

    # ---------------------- Packet animation ----------------------

    def send_packet(self):
        # Determine source and destination devices
        selected = [it for it in self.scene.selectedItems() if isinstance(it, DraggableDeviceItem)]
        if len(selected) >= 2:
            src_item: DraggableDeviceItem = selected[0]
            dst_item: DraggableDeviceItem = selected[1]
        else:
            # fallback to first two devices
            if len(self.id_to_item) < 2:
                QMessageBox.information(self, "Send Packet", "Need at least two devices (or select two).")
                return
            ids_sorted = sorted(self.id_to_item.keys())
            src_item = self.id_to_item[ids_sorted[0]]
            dst_item = self.id_to_item[ids_sorted[1]]

        src_id = self.item_to_id[src_item]
        dst_id = self.item_to_id[dst_item]
        src_ip = self.device_registry[src_id]["ip"]
        dst_ip = self.device_registry[dst_id]["ip"]

        # Log to console
        print(f"Sending packet from {self.device_registry[src_id]['label']} ({src_ip}) to {self.device_registry[dst_id]['label']} ({dst_ip})")

        start = src_item.sceneBoundingRect().center()
        end = dst_item.sceneBoundingRect().center()

        # Packet visual
        radius = 6.0
        packet = QGraphicsEllipseItem(-radius, -radius, radius * 2, radius * 2)
        packet.setBrush(Qt.GlobalColor.red)
        packet.setPen(Qt.GlobalColor.black)
        packet.setZValue(1000)
        self.scene.addItem(packet)
        packet.setPos(start)

        anim = QVariantAnimation(self)
        anim.setStartValue(0.0)
        anim.setEndValue(1.0)
        anim.setDuration(1200)

        def on_value_changed(v):
            t = float(v)
            x = start.x() + (end.x() - start.x()) * t
            y = start.y() + (end.y() - start.y()) * t
            packet.setPos(QPointF(x, y))

        def on_finished():
            # Show popup with packet info
            QMessageBox.information(
                self,
                "Packet Arrived",
                f"Source IP: {src_ip}\nDestination IP: {dst_ip}"
            )
            # Clean up visual
            self.scene.removeItem(packet)

        anim.valueChanged.connect(on_value_changed)
        anim.finished.connect(on_finished)
        anim.start()
        # Keep a reference to avoid garbage collection
        if not hasattr(self, "_active_anims"):
            self._active_anims = []
        self._active_anims.append(anim)
        def cleanup_anim():
            if anim in self._active_anims:
                self._active_anims.remove(anim)
        anim.finished.connect(cleanup_anim)


# ---------------------- CLI SECTION ----------------------

from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field

from prompt_toolkit import PromptSession
from prompt_toolkit.completion import Completer, Completion
from prompt_toolkit.formatted_text import HTML
from prompt_toolkit.history import InMemoryHistory


@dataclass
class InterfaceConfig:
    name: str
    ip_address: Optional[Tuple[str, str]] = None  # (ip, mask)


@dataclass
class DeviceConfig:
    hostname: str = "ciskosim"
    interfaces: Dict[str, InterfaceConfig] = field(default_factory=dict)

    def get_running_config_lines(self) -> List[str]:
        lines: List[str] = [f"hostname {self.hostname}"]
        for if_name, iface in self.interfaces.items():
            lines.append(f"interface {if_name}")
            if iface.ip_address:
                ip, mask = iface.ip_address
                lines.append(f" ip address {ip} {mask}")
            lines.append(" exit")
        return lines


class Mode:
    USER = ">"
    ENABLE = "#"
    CONFIG = "(config)#"
    IFACE = "(config-if)#"


class CiscoLikeCompleter(Completer):
    def __init__(self, mode: str, device: DeviceConfig, current_interface: Optional[str]):
        self.mode = mode
        self.device = device
        self.current_interface = current_interface

    def get_completions(self, document, complete_event):
        text = document.text_before_cursor.strip()
        parts = text.split()

        def complete_word(candidates: List[str]):
            word = parts[-1] if parts else ""
            for c in candidates:
                if c.startswith(word):
                    yield Completion(c, start_position=-len(word))

        if self.mode == Mode.USER:
            if not parts:
                for c in ["enable", "show"]:
                    yield Completion(c, start_position=0)
            elif parts[0] == "show":
                yield from complete_word(["running-config", "ip"])
            elif parts[0] == "ip":
                pass
            else:
                yield from complete_word(["enable", "show"])        
        elif self.mode == Mode.ENABLE:
            if not parts:
                for c in ["disable", "configure", "show"]:
                    yield Completion(c, start_position=0)
            elif parts[0] == "configure":
                yield from complete_word(["terminal"])
            elif parts[0] == "show":
                yield from complete_word(["running-config", "ip"])
            elif parts[0] == "ip":
                pass
            else:
                yield from complete_word(["disable", "configure", "show"])        
        elif self.mode == Mode.CONFIG:
            if not parts:
                for c in ["exit", "interface", "hostname"]:
                    yield Completion(c, start_position=0)
            elif parts[0] == "interface":
                # suggest existing and common names
                candidates = list(self.device.interfaces.keys()) or [
                    "GigabitEthernet0/0",
                    "GigabitEthernet0/1",
                    "Loopback0",
                ]
                yield from complete_word(candidates)
            elif parts[0] == "hostname":
                pass
            else:
                yield from complete_word(["exit", "interface", "hostname"])        
        elif self.mode == Mode.IFACE:
            if not parts:
                for c in ["exit", "ip", "shutdown", "no"]:
                    yield Completion(c, start_position=0)
            elif parts[0] == "ip":
                yield from complete_word(["address"])
            elif parts[0] == "no":
                yield from complete_word(["shutdown"])
            else:
                yield from complete_word(["exit", "ip", "shutdown", "no"])        


class CiscoLikeCLI:
    def __init__(self):
        self.device = DeviceConfig()
        self.mode = Mode.USER
        self.current_interface: Optional[str] = None
        self.history = InMemoryHistory()
        self.session = PromptSession(history=self.history)

    def _prompt(self) -> str:
        if self.mode == Mode.USER:
            p = f"{self.device.hostname}> "
        elif self.mode == Mode.ENABLE:
            p = f"{self.device.hostname}# "
        elif self.mode == Mode.CONFIG:
            p = f"{self.device.hostname}(config)# "
        else:
            p = f"{self.device.hostname}(config-if)# "
        return p

    def _completer(self) -> Completer:
        return CiscoLikeCompleter(self.mode, self.device, self.current_interface)

    def run(self):
        while True:
            try:
                text = self.session.prompt(HTML(f"<b>{self._prompt()}</b>"), completer=self._completer())
            except (EOFError, KeyboardInterrupt):
                print("Exiting CLI.")
                break
            line = text.strip()
            if not line:
                continue
            if line in ("quit", "exit") and self.mode == Mode.USER:
                break
            self._handle_command(line)

    # ---------------------- command handlers ----------------------

    def _handle_command(self, line: str):
        parts = line.split()
        cmd = parts[0]
        args = parts[1:]

        if self.mode == Mode.USER:
            if cmd == "enable":
                self.mode = Mode.ENABLE
            elif cmd == "show":
                self._cmd_show(args)
            else:
                print("% Invalid input detected at '^' marker.")
        elif self.mode == Mode.ENABLE:
            if cmd == "disable":
                self.mode = Mode.USER
            elif cmd == "configure" and args[:1] == ["terminal"]:
                self.mode = Mode.CONFIG
            elif cmd == "show":
                self._cmd_show(args)
            else:
                print("% Invalid input detected at '^' marker.")
        elif self.mode == Mode.CONFIG:
            if cmd == "exit":
                self.mode = Mode.ENABLE
            elif cmd == "hostname" and args:
                self.device.hostname = args[0]
            elif cmd == "interface" and args:
                if_name = args[0]
                if if_name not in self.device.interfaces:
                    self.device.interfaces[if_name] = InterfaceConfig(name=if_name)
                self.current_interface = if_name
                self.mode = Mode.IFACE
            else:
                print("% Invalid input detected at '^' marker.")
        elif self.mode == Mode.IFACE:
            if cmd == "exit":
                self.current_interface = None
                self.mode = Mode.CONFIG
            elif cmd == "ip" and args[:1] == ["address"] and len(args) >= 3:
                ip, mask = args[1], args[2]
                assert self.current_interface is not None
                self.device.interfaces[self.current_interface].ip_address = (ip, mask)
            elif cmd == "shutdown":
                # placeholder, not stored
                print("Interface administratively down (not persisted in this demo)")
            elif cmd == "no" and args[:1] == ["shutdown"]:
                print("Interface up (not persisted in this demo)")
            else:
                print("% Invalid input detected at '^' marker.")

    def _cmd_show(self, args: List[str]):
        if not args:
            print("% Incomplete command.")
            return
        if args[0] == "running-config":
            for line in self.device.get_running_config_lines():
                print(line)
        elif args[0] == "ip":
            if len(args) >= 3 and args[1] == "interface" and args[2] == "brief":
                print("Interface\tIP-Address\tMask")
                for if_name, iface in self.device.interfaces.items():
                    ip_str = iface.ip_address[0] if iface.ip_address else "unassigned"
                    mask_str = iface.ip_address[1] if iface.ip_address else "-"
                    print(f"{if_name}\t{ip_str}\t{mask_str}")
            else:
                print("% Incomplete command.")
        else:
            print("% Invalid input detected at '^' marker.")


# ---------------------- ENTRYPOINT ----------------------

def run_gui():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


def run_cli():
    cli = CiscoLikeCLI()
    cli.run()


if __name__ == "__main__":
    if "--cli" in sys.argv:
        # Lazy import prompt_toolkit dependency hint
        try:
            run_cli()
        except ModuleNotFoundError as e:
            print("prompt_toolkit is required for CLI mode. Install with: pip install prompt_toolkit")
            raise
    else:
        run_gui()

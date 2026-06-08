import subprocess

from savo_realsense.constants import DEFAULT_USB_PRODUCT_HINT, DEFAULT_USB_VENDOR_ID


def list_usb_devices() -> list[str]:
    result = subprocess.run(
        ["lsusb"],
        check=False,
        capture_output=True,
        text=True,
    )

    if result.returncode != 0:
        return []

    return [
        line.strip()
        for line in result.stdout.splitlines()
        if line.strip()
    ]


def realsense_usb_lines(
    usb_lines: list[str],
    vendor_id: str = DEFAULT_USB_VENDOR_ID,
    product_hint: str = DEFAULT_USB_PRODUCT_HINT,
) -> list[str]:
    hint = product_hint.lower()

    return [
        line
        for line in usb_lines
        if vendor_id.lower() in line.lower() or hint in line.lower()
    ]


def realsense_usb_detected(
    usb_lines: list[str],
    vendor_id: str = DEFAULT_USB_VENDOR_ID,
    product_hint: str = DEFAULT_USB_PRODUCT_HINT,
) -> bool:
    return bool(realsense_usb_lines(usb_lines, vendor_id, product_hint))


def format_usb_report(usb_lines: list[str]) -> str:
    matches = realsense_usb_lines(usb_lines)

    if matches:
        lines = ["RealSense USB device detected:"]
        lines.extend(f"- {line}" for line in matches)
        return "\n".join(lines)

    return "No RealSense USB device detected"
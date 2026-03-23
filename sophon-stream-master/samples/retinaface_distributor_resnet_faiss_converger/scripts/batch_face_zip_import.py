#!/usr/bin/env python3
import argparse
import json
import os
import re
import sys
import zipfile
from collections import OrderedDict


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
VALID_FACE_NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,64}$")


def is_supported_image_file(filename: str) -> bool:
    _, ext = os.path.splitext(filename)
    return ext.lower() in IMAGE_EXTS


def sanitize_upload_filename(raw_filename: str, index_seed: int) -> str:
    filename = os.path.basename(raw_filename or "")
    sanitized = []
    for ch in filename:
        if ch.isalnum() or ch in "._-":
            sanitized.append(ch)
        else:
            sanitized.append("_")
    sanitized_name = "".join(sanitized)
    if not sanitized_name or sanitized_name in {".", ".."}:
        sanitized_name = f"face_{index_seed}.jpg"
    elif not is_supported_image_file(sanitized_name):
        sanitized_name += ".jpg"
    return sanitized_name


def make_unique_path(folder: str, filename: str) -> str:
    root, ext = os.path.splitext(filename)
    candidate = os.path.join(folder, filename)
    suffix = 1
    while os.path.exists(candidate):
        candidate = os.path.join(folder, f"{root}_{suffix}{ext}")
        suffix += 1
    return candidate


def normalize_parts(zip_name: str):
    normalized = (zip_name or "").replace("\\", "/")
    raw_parts = [part for part in normalized.split("/") if part not in ("", ".")]
    for part in raw_parts:
        if part == "..":
            raise ValueError("zip path contains invalid parent directory reference")
    return raw_parts


def detect_wrapper_depth(path_parts_list):
    top_levels = []
    seen = set()
    for parts in path_parts_list:
        if not parts:
            continue
        if parts[0] not in seen:
            top_levels.append(parts[0])
            seen.add(parts[0])
    if len(top_levels) != 1:
        return 0
    has_second_level_faces = False
    for parts in path_parts_list:
        if len(parts) >= 3 and VALID_FACE_NAME_RE.fullmatch(parts[1]):
            has_second_level_faces = True
            break
    return 1 if has_second_level_faces else 0


def append_warning(warnings, message):
    if len(warnings) < 80:
        warnings.append(message)


def build_result(status="success", message="ok"):
    return {
        "status": status,
        "message": message,
        "total_saved": 0,
        "total_faces": 0,
        "faces": [],
        "warnings": [],
    }


def main():
    parser = argparse.ArgumentParser(description="Batch import face images from zip archive.")
    parser.add_argument("--zip", dest="zip_path", required=True, help="Path to zip archive")
    parser.add_argument("--train-dir", dest="train_dir", required=True, help="Face training directory")
    parser.add_argument("--result", dest="result_path", required=True, help="Output result json path")
    args = parser.parse_args()

    result = build_result()

    try:
        os.makedirs(args.train_dir, exist_ok=True)
        if not os.path.isfile(args.zip_path):
            raise RuntimeError("zip archive not found")

        with zipfile.ZipFile(args.zip_path, "r") as archive:
            file_infos = [info for info in archive.infolist() if not info.is_dir()]
            if not file_infos:
                raise RuntimeError("zip archive is empty")

            parts_list = []
            for info in file_infos:
                try:
                    parts = normalize_parts(info.filename)
                except ValueError as exc:
                    append_warning(result["warnings"], f"已忽略非法路径: {info.filename} ({exc})")
                    continue
                if parts:
                    parts_list.append(parts)

            wrapper_depth = detect_wrapper_depth(parts_list)
            imported_faces = OrderedDict()
            seed = 1

            for info in file_infos:
                try:
                    parts = normalize_parts(info.filename)
                except ValueError:
                    continue

                if len(parts) < wrapper_depth + 2:
                    append_warning(result["warnings"], f"已忽略未按文件夹归类的文件: {info.filename}")
                    continue

                face_name = parts[wrapper_depth]
                original_folder = face_name
                if not VALID_FACE_NAME_RE.fullmatch(face_name):
                    append_warning(
                        result["warnings"],
                        f"已忽略文件夹 {original_folder}: 名称仅支持英文、数字、下划线和短横线",
                    )
                    continue

                filename = parts[-1]
                if not is_supported_image_file(filename):
                    append_warning(result["warnings"], f"已忽略非图片文件: {info.filename}")
                    continue

                target_dir = os.path.join(args.train_dir, face_name)
                os.makedirs(target_dir, exist_ok=True)
                safe_name = sanitize_upload_filename(filename, seed)
                target_path = make_unique_path(target_dir, safe_name)

                with archive.open(info, "r") as src, open(target_path, "wb") as dst:
                    data = src.read()
                    if not data:
                        append_warning(result["warnings"], f"已忽略空文件: {info.filename}")
                        continue
                    dst.write(data)

                face_entry = imported_faces.setdefault(face_name, {"name": face_name, "savedCount": 0})
                face_entry["savedCount"] += 1
                result["total_saved"] += 1
                seed += 1

            result["faces"] = list(imported_faces.values())
            result["total_faces"] = len(result["faces"])

            if result["total_saved"] <= 0:
                raise RuntimeError("压缩包中未找到符合要求的英文文件夹图片数据")

    except zipfile.BadZipFile:
        result = build_result(status="error", message="invalid zip archive")
    except Exception as exc:
        if result.get("status") != "error":
            result["status"] = "error"
            result["message"] = str(exc)

    with open(args.result_path, "w", encoding="utf-8") as fh:
        json.dump(result, fh, ensure_ascii=False, indent=2)

    return 0 if result.get("status") == "success" else 1


if __name__ == "__main__":
    sys.exit(main())

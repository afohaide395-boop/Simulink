"""打包模型、真实端口重连脚本和匹配的接线清单。"""
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED

HERE = Path(__file__).resolve().parent
FILES = ['reduce_emf_checked.slx', 'repair_organized_wires.m',
         'organized_wiring_manifest.json', 'checked_model_readme.txt']


def main():
    output = HERE / 'reduce_emf_checked_package.zip'
    with ZipFile(output, 'w', compression=ZIP_DEFLATED) as archive:
        for name in FILES:
            archive.write(HERE / name, arcname=name)
    with ZipFile(output) as archive:
        assert archive.testzip() is None
        for name in FILES:
            assert archive.read(name) == (HERE / name).read_bytes()
    print(output)


if __name__ == '__main__':
    main()

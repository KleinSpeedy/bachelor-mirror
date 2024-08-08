# Bachelor-thesis

Bachelorarbeit 2024

## Usage

**Build:**
```sh
./skripte/2pdf.sh
```

**Clean manually:**
```sh
./skripte/clean.sh
```

**Build docker image**
```sh
docker build -t thesis-builder .
```

**Build using docker**
```sh
docker run --rm -it -v $(pwd):/bachelor-thesis thesis-builder ./skripte/2pdf.sh
```

## Implementation repositories

Instead of adding all repositories as submodules they are linked here to avoid
a massive git tree:

* [**Qemu EDI extension**](https://github.com/KleinSpeedy/qemu-extension/tree/thesis-code-dive-extension)
* [**Qemu device extension**](https://github.com/KleinSpeedy/qemu-extension/tree/thesis-device-extension)
* [**GPIO test project**]()
* [**UART test project**]()

## Automatic PDF build

With every push to **main** a pipeline-job is triggered.
The pdf is built inside the CI using the `2pdf.sh` script and the `Dockerfile`.
After each successful built a release is created and tagged with the *Unix
timestamp*.

To inspect the Release Tag use the `date` utility:
```sh
date -d @$RELEASE_TAG
```

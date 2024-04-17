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

## Automatic PDF build

With every push to **main** a pipeline-job is triggered.
The pdf is built inside the CI using the `2pdf.sh` script and the `Dockerfile`.
After each successful built a release is created and tagged with the *Unix
timestamp*.

To inspect the Release Tag use the `date` utility:
```sh
date -d @$RELEASE_TAG
```

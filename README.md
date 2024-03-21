# 2024 J.Schulze - Emulation eines 32bit Mikrocontrollers

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

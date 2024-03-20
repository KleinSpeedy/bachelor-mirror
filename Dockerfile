FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update

RUN apt-get install -y \
    build-essential \
    texlive texlive-base texlive-luatex  \
    texlive-lang-german texlive-lang-english \
    texlive-fonts-extra \
    inkscape biber

WORKDIR /bachelor-thesis

FROM russtedrake/manipulation:db98d5c as manipulation
FROM robotlocomotion/drake:focal-20211129 as drake

RUN apt-get update && apt-get upgrade -y && apt install -y \
    build-essential \
    nginx \
    python3-dev \
    python3-venv \
    xvfb

ADD https://github.com/krallin/tini/releases/download/v0.19.0/tini /usr/bin/tini
RUN chmod +x /usr/bin/tini

ARG UNAME=user
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID -o $UNAME
RUN useradd -m -u $UID -g $GID -o $UNAME && \
  mkdir -p /jupyter && \
  chown -R $UNAME:$UNAME /jupyter
USER $UNAME

WORKDIR /jupyter
RUN python -m venv .venv
COPY requirements.txt .
RUN .venv/bin/pip install -r requirements.txt

COPY --from=manipulation /opt/manipulation/manipulation/ .venv/lib/python3.8/site-packages/manipulation

RUN mkdir -p /jupyter/notebooks

# default port for jupyter
EXPOSE 8888
# meshcat will use port 7000 first, then increment by 1 for each subsequent instantiation
EXPOSE 7000-7100

ENTRYPOINT ["/usr/bin/tini", "--"]
CMD [".venv/bin/jupyter", "lab", "--no-browser", "--ip=0.0.0.0", "--notebook-dir=/jupyter/notebooks"]

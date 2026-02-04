# Runtime with glibc >= 2.38
FROM --platform=$TARGETPLATFORM ubuntu:24.04

# Minimal runtime deps (libstdc++ just in case the binary was compiled with g++)
RUN apt-get update \
 && apt-get install -y --no-install-recommends libstdc++6 ca-certificates \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY run_loop_arm /app/run_loop
RUN chmod +x /app/run_loop

# If your app needs env vars or data files, add them here
# ENV FOO=bar

ENTRYPOINT ["/app/run_loop"]

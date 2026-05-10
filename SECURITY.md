# Security Policy

## Reporting a Vulnerability

If you believe you have found a security vulnerability in libde265, **please do
not open a public GitHub issue**. Instead, report it privately via one of the
following channels:

- **Preferred:** Use GitHub's private vulnerability reporting by clicking
  *Report a vulnerability* on the
  [Security tab](https://github.com/strukturag/libde265/security/advisories/new)
  of this repository. This creates a private draft advisory that only the
  maintainers can see.
- **Alternative:** If you cannot use GitHub, send an email to
  <dirk.farin@gmail.com> describing the issue.

When reporting, please include as much of the following as you can:

- A description of the vulnerability and its potential impact.
- Steps to reproduce, ideally with a minimal H.265 bitstream that triggers the
  issue.
- The libde265 version (release tag or commit hash) and build configuration
  (compiler, sanitizers, etc.) where the issue was observed.
- Any suggested fix or mitigation, if known.

We will acknowledge your report, work with you on a fix, and credit you in the
release notes and advisory unless you prefer to remain anonymous.

## Supported Versions

Security fixes are applied to the latest release on the `master` branch. Older
releases are not maintained; please update to the current release before
reporting.

## Scope

In scope:

- Memory safety issues in the decoder (out-of-bounds reads/writes, use-after-
  free, double-free, uninitialized memory use).
- Crashes, hangs, or excessive resource consumption triggered by crafted H.265
  bitstreams.
- Issues in the public C API (`libde265/de265.h`) that lead to the above when
  used as documented.

Out of scope:

- Bugs in the experimental encoder (`libde265/encoder/`, built with
  `-DENABLE_ENCODER=ON`).
- Bugs in the sample applications (`dec265/`, `enc265/`, `sherlock265/`) that
  do not also affect the library.
- Issues that require a non-default, unsafe build configuration or modified
  source.

## Disclosure

We aim to coordinate disclosure with the reporter. Once a fix is available, we
will publish a GitHub Security Advisory and request a CVE where appropriate.

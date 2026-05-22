# Datasheets

Manufacturer datasheets for the **custom-driver ICs** in this project — the ones we
wrote/ported a register-level driver for and therefore need to validate against the
silicon (see `plans/transient-issues-fixes.md`, findings F14–F16).

| File | Part | Vendor | Used by |
|------|------|--------|---------|
| `bq25620-bq25622-ti.pdf` | BQ25620 / BQ25622 (1-cell buck battery charger) | Texas Instruments | `src/bq2562x.*` |
| `sx1280-sx1281-semtech.pdf` | SX1280 / SX1281 (2.4 GHz LoRa/FLRC transceiver) | Semtech | `lib/SX128xLink` |

ICs that use mature vendor libraries (OLED `SH110X`, `ADS1115` via Adafruit; SX128x
low-level via RadioLib) are intentionally **not** archived here — their register
sequencing is library-managed, not hand-written.

## Re-downloading

```bash
./scripts/fetch-datasheets.sh
```

The script handles the two reasons a plain HTTP fetch fails on these:

- **TI (and most vendors: ADI, ST, NXP, Microchip):** the CDN 403s any non-browser
  User-Agent. Sending a normal browser UA is enough — TI's `ti.com/lit/ds/symlink/`
  PDFs are static.
- **Semtech:** no static PDF. The datasheet link is a Salesforce content-distribution
  page that sets a session cookie, JS-POSTs back, then serves the file from the
  `renditionDownload` servlet. The script replays that flow with a cookie jar. The
  `versionId`/`contentId`/`oid` are pinned in the script; if Semtech republishes the
  doc, refresh them from the viewer's network requests (see the comment in the script).

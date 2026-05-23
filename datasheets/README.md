# Datasheets

Manufacturer datasheets for the radio ICs used by this project.

| File | Part | Vendor | Used by |
|------|------|--------|---------|
| `sx1280-sx1281-semtech.pdf` | SX1280 / SX1281 (2.4 GHz LoRa/FLRC transceiver) | Semtech | `lib/xlrs/phy/` |

## Re-downloading

```bash
./scripts/fetch-datasheets.sh
```

The script handles the reason a plain HTTP fetch fails for Semtech:

- **Semtech:** no static PDF. The datasheet link is a Salesforce content-distribution
  page that sets a session cookie, JS-POSTs back, then serves the file from the
  `renditionDownload` servlet. The script replays that flow with a cookie jar. The
  `versionId`/`contentId`/`oid` are pinned in the script; if Semtech republishes the
  doc, refresh them from the viewer's network requests (see the comment in the script).

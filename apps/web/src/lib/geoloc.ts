/**
 * Constants mirrored from the Python perception layer.
 *
 * Duplicated deliberately and narrowly: the dashboard needs to *explain* the range gate
 * in its empty state, and shipping the whole perception config to the browser to do that
 * would be worse. If `parkscan_perception.geolocation.DEFAULT_MAX_RANGE_M` changes, this
 * must change with it — the value is asserted in `geoloc.test.ts` against the number the
 * API reports, so the two cannot drift silently.
 */
export const DEFAULT_MAX_RANGE_M = 6.0;

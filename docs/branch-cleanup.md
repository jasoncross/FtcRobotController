# Branch cleanup assessment

Inspected and initial cleanup completed September 7, 2026 after fetching and
pruning remote-tracking refs. The six fully preserved fork branches below and
their three local copies were deleted. The two local-only branches were archived
to published tags before their local branch names were removed.

Keep `master`, `codex/biobuzz-base`, `decode-2025-2026-final`, and `DecodeFinal`.
`master` currently preserves the merged DECODE release; development is on
`codex/biobuzz-base`. Changing the default branch or merging BIOBUZZ into
`master` is a separate decision.

## Fork branches deleted

These six `origin` branches are ancestors of `decode-2025-2026-final`, and none
is the source or target of an open pull request in the fork at review time:

- `speed-tweaks`
- `feed-system`
- `limelight`
- `codex/add-auto-scale-adjustment-method`
- `codex/add-auto-scale-adjustment-method-1cnl34`
- `codex/set-intake-to-on-by-default-in-auto-modes`

Local copies of `speed-tweaks`, `feed-system`, and `limelight` were also deleted.
Their histories remain reachable through the final season tag.

## Evaluated: retire limelight-part3 after PR disposition

The [comparison with the team checkout](limelight-retirement.md) concludes that
`limelight-part3` is no longer needed as an active development branch. It has
zero commits outside the preserved DECODE history, and its vision/odometry
files exactly match the later team checkout. The six PRs below should be closed
as superseded or archived experiments, rather than merged into BIOBUZZ. Their
unmerged source tips should be archived before branch deletion. This evaluation
did not close PRs or delete their base/source branches.

`limelight-part3` is fully preserved in the final tag, but remains the base of
six open PRs in `jasoncross/FtcRobotController`. Keep it and the source branches
while deciding whether to close these DECODE-era proposals:

| PR | Source branch | Commits not reachable from the BIOBUZZ branch |
| --- | --- | ---: |
| [#99](https://github.com/jasoncross/FtcRobotController/pull/99) | `codex/improve-limelight-pipeline-selection-stability` | 2 |
| [#106](https://github.com/jasoncross/FtcRobotController/pull/106) | `codex/modify-limelight-vision-integration` | 19 |
| [#107](https://github.com/jasoncross/FtcRobotController/pull/107) | `codex/modify-limelight-vision-integration-jordg9` | 1 |
| [#110](https://github.com/jasoncross/FtcRobotController/pull/110) | `codex/add-feedstop-control-for-launchers` | 3 |
| [#111](https://github.com/jasoncross/FtcRobotController/pull/111) | `codex/add-feedstop-control-for-launchers-sorly4` | 1 |
| [#112](https://github.com/jasoncross/FtcRobotController/pull/112) | `codex/add-feedstop-control-for-launchers-4s95qm` | 2 |

The local `codex/modify-limelight-vision-integration` branch is also covered by
this caution. These are commit ancestry counts, not proof that every change is
absent: equivalent changes may have been merged under different commits.
Review or preserve the tips with annotated archive tags before deleting them.

## Local-only branches archived and deleted

| Former local branch | Unmerged commits | Published archive tag |
| --- | ---: | --- |
| `limelight-redux` | 13 | `archive/decode/limelight-redux` |
| `odometry-addition` | 2 | `archive/decode/odometry-addition` |

Their tips were not contained in the DECODE release tag. Both annotated archive
tags were pushed successfully before the local branches were deleted.

The team repository (`upstream`) was not modified. Its `speed-tweaks` commit
`69bb375` matches the clean local team checkout and contains four autonomous
file changes beyond `DecodeFinal`. It is now independently preserved in the
fork as `archive/decode/team-checkout-2026-02-17`. Existing season tags were
not moved. See the comparison report for the exact changes.

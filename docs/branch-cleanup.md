# Branch cleanup assessment

Inspected September 7, 2026 after fetching and pruning remote-tracking refs.
No local or remote branches were deleted as part of this assessment.

Keep `master`, `codex/biobuzz-base`, `decode-2025-2026-final`, and `DecodeFinal`.
`master` currently preserves the merged DECODE release; development is on
`codex/biobuzz-base`. Changing the default branch or merging BIOBUZZ into
`master` is a separate decision.

## Fork branches ready for deletion

These six `origin` branches are ancestors of `decode-2025-2026-final`, and none
is the source or target of an open pull request in the fork at review time:

- `speed-tweaks`
- `feed-system`
- `limelight`
- `codex/add-auto-scale-adjustment-method`
- `codex/add-auto-scale-adjustment-method-1cnl34`
- `codex/set-intake-to-on-by-default-in-auto-modes`

Local copies of `speed-tweaks`, `feed-system`, and `limelight` are also fully
preserved. Deleting these branch names would not remove their history from the
final season tag. Refresh refs and recheck open PRs before executing cleanup.

## Keep until the old PRs are resolved

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

## Local branches whose remote branches are gone

| Local branch | Commits not reachable from the BIOBUZZ branch |
| --- | ---: |
| `limelight-redux` | 13 |
| `odometry-addition` | 2 |

Their tips are not contained in the DECODE release tag either. Preserve them
with archive tags or review their changes before deleting their last branch refs.

The team repository (`upstream`) is outside the proposed fork cleanup. In
particular, `upstream/speed-tweaks` has one commit outside the BIOBUZZ history;
do not assume all team branches have been preserved by this transition.

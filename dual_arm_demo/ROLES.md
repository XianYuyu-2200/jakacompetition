# JAKA Mini 2 Role Assignment

Final role assignment:

```text
operator arm: 192.168.0.101
tracking arm: 192.168.0.102
data flow: 192.168.0.101 -> 192.168.0.102
```

The operator arm is moved by the human. The tracking arm will eventually
follow it, but the current dry-run program only reads state and calculates
targets. It does not send motion commands.

Run the read-only dry-run with:

```bash
cd ~/codex/codex-competition/dual_arm_demo
./run_operator_tracking_dry_run.sh
```

Use `Ctrl+C` to stop both processes.

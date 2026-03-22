import re

with open('src/balance_bot/behavior/agent.py', 'r') as f:
    content = f.read()

# I accidentally duplicated the "Check background tasks (Saving)" and "Update Battery Logic" blocks
# when applying the re.sub for the new_loop_start. Let's clean that up.
dup_block = """                # Check background tasks (Saving)
                if self.ticks % 10 == 0:
                    self.led.update()
                    if self.config_dirty and (time.monotonic() - self.last_save_time > self.learning_state.timing.save_interval):
                        try:
                            config_snapshot = self.learning_state.model_dump()
                            self.io_executor.submit(self._save_config_worker, config_snapshot)
                            self.last_save_time = time.monotonic()
                            self.config_dirty = False
                        except Exception as e:
                            logger.error(f"Failed to initiate async config save: {e}")

                # Update Battery Logic (Always run to keep voltage filter updated)
                if last_telemetry:
                    ang_accel = (last_telemetry.pitch_rate - last_pitch_rate) / dt
                    last_pitch_rate = last_telemetry.pitch_rate
                    _comp_factor = self.battery.update(last_telemetry.motor_output, ang_accel)
                    if _comp_factor < self.learning_state.control.low_battery_log_threshold and self.battery_logger.should_log():
                        logger.warning(f"-> Low Battery? Compensating: {int(_comp_factor * 100)}%")
                else:
                    # Fallback if no telemetry (e.g. after Kickup)
                    pass

"""
content = content.replace(dup_block + dup_block, dup_block)

# Wait, maybe they are separated by something?
content = re.sub(
    r'                # Check background tasks \(Saving\).*?                    pass\n\n                # Check background tasks \(Saving\).*?                    pass\n',
    dup_block,
    content,
    flags=re.DOTALL
)

with open('src/balance_bot/behavior/agent.py', 'w') as f:
    f.write(content)


with open('tests/test_startup_logic.py', 'r') as f:
    content = f.read()

content = content.replace("agent.core.update.return_value = None", "agent.core.update.return_value = MagicMock()")

with open('tests/test_startup_logic.py', 'w') as f:
    f.write(content)

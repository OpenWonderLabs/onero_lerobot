"""Run native LeRobot PI05 with pre-action contract and loaded-weight guards."""

import logging
from importlib.metadata import version


def run_verified_rollout(cfg):
    from lerobot.rollout import build_rollout_context, create_strategy
    from lerobot.utils.process import ProcessSignalHandler
    from lerobot.utils.utils import init_logging
    from lerobot.utils.visualization_utils import init_visualization, shutdown_visualization

    from onero_h1_lerobot.pi05_inference import prepare_pi05_rollout, verify_loaded_pi05_weights

    if version("lerobot") != "0.6.1":
        raise RuntimeError("Guarded native PI05 rollout is qualified for LeRobot 0.6.1")
    checkpoint = prepare_pi05_rollout(cfg)
    init_logging()
    handler = ProcessSignalHandler(use_threads=True, display_pid=False)
    context = None
    strategy = None
    verified = False
    try:
        if cfg.display_data:
            init_visualization(
                cfg.display_mode,
                session_name="rollout",
                ip=cfg.display_ip,
                port=cfg.display_port,
            )
        context = build_rollout_context(cfg, handler.shutdown_event)
        summary = verify_loaded_pi05_weights(context.policy.policy, checkpoint)
        logging.info("PI05 loaded-weight verification passed: %s", summary)
        verified = True
        strategy = create_strategy(cfg.strategy)
        strategy.setup(context)
        strategy.run(context)
    except KeyboardInterrupt:
        logging.info("Interrupted by user")
    finally:
        if context is not None:
            if verified and strategy is not None:
                strategy.teardown(context)
            else:
                # A failed weight guard must never trigger native return-to-pose.
                try:
                    context.policy.inference.stop()
                finally:
                    try:
                        if context.hardware.teleop is not None:
                            context.hardware.teleop.disconnect()
                    finally:
                        context.hardware.robot_wrapper.inner.disconnect()
        if cfg.display_data:
            shutdown_visualization(cfg.display_mode)
    logging.info("Rollout finished")


def main():
    from lerobot.configs import parser
    from lerobot.rollout import RolloutConfig
    from lerobot.utils.import_utils import register_third_party_plugins

    import onero_h1_lerobot  # noqa: F401

    register_third_party_plugins()

    # Keep the upstream config/parser and all strategy/inference implementations.
    # No future annotations: the parser must receive the actual config class.
    @parser.wrap()
    def run(cfg: RolloutConfig):
        run_verified_rollout(cfg)

    run()


if __name__ == "__main__":
    main()

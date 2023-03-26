defmodule FleetMonitorTest do
  use ExUnit.Case
  doctest FleetMonitor

  test "greets the world" do
    assert FleetMonitor.hello() == :world
  end
end

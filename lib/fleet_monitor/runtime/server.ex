defmodule FleetMonitor.Runtime.Server do
  use GenServer
  require Logger

  @type t :: pid

  @command "python3.8 ./lib/python/test_count.py"

  ### Client Process

  def start_link(_) do
    GenServer.start_link(__MODULE__, nil)
  end

  ### Server Process
  def init(_) do
    _port = Port.open({:spawn, @command}, [:binary, :exit_status])

    { :ok, %{ latest_output: nil, exit_status: nil }}
  end

  def handle_info({ _port, { :data, data }}, state) do
    Logger.info "Latest output: #{data}"
    { :noreply, %{ state | latest_output: data }}
  end

  def handle_info({ _port, { :exit_status, status}}, state) do
    Logger.info "External exit: :exit_status: #{status}"

    { :noreply, %{ state | exit_status: status }}
  end

  def handle_info(_msg, state), do: { :noreply, state }
end

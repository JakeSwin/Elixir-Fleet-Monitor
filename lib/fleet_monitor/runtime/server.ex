defmodule FleetMonitor.Runtime.Server do
  use GenServer
  require Logger

  @type t :: pid

  @command "python3.8 ./lib/python/test_string.py"

  ### Client Process

  def start_link(_) do
    GenServer.start_link(__MODULE__, nil)
  end

  ### Server Process
  def init(_) do
    port = Port.open({:spawn, @command}, [:binary, :exit_status])

    { :ok, %{ latest_output: nil, exit_status: nil }}
  end

  def handle_info({ port, { :data, data }}, state) do
    cleaned_data =
      data
      |> String.trim()

    Logger.info "Latest output: #{cleaned_data}"
    { :noreply, %{ state | latest_output: cleaned_data }}
  end

  def handle_info({ port, { :exit_status, status}}, state) do
    Logger.info "External exit: :exit_status: #{status}"

    new_state = %{ state | exit_status: status }
    { :noreply, %{ state | exit_status: status }}
  end

  def handle_info(_msg, state), do: { :noreply, state }
end
